#!/usr/bin/env python3
"""
ROS2 fuzzy middleware for PTO-aware speed scaling.

Flow:
  /cmd_vel_raw + (/pto/rpm, /pto/current, /extra_motor_status) -> /cmd_vel

This node keeps angular command unchanged and scales only linear.x.
It uses raw PTO/KLS motor RPM, so logs and fuzzy control work with the same
physical value. The fuzzy rules are constrained by a hard RPM envelope:
  - rpm >= rpm_full_speed: full speed
  - rpm_stop <= rpm < rpm_full_speed: linear speed reduction
  - rpm < rpm_stop: robot stop
  - rpm < rpm_shutdown for rpm_shutdown_timeout_sec: command PTO off

The node treats missing/zero PTO telemetry as a drive stop only while the
PTO implement is commanded active. That prevents a normal software PTO-off
state from stopping the robot, but stops the robot if the PTO
battery/controller disappears while the implement is active.
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Int16, String, UInt64
from geometry_msgs.msg import Twist


DEFAULT_CURRENT_DERATE_AMPS = [190.0, 220.0, 250.0, 280.0, 300.0]
DEFAULT_CURRENT_DERATE_FACTORS = [1.0, 0.9, 0.8, 0.5, 0.0]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def trimf(x: float, a: float, b: float, c: float) -> float:
    if a == b:
        if x <= b:
            return 1.0
        if x >= c:
            return 0.0
        den = c - b
        return 0.0 if den == 0 else (c - x) / den
    if b == c:
        if x >= b:
            return 1.0
        if x <= a:
            return 0.0
        den = b - a
        return 0.0 if den == 0 else (x - a) / den
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        den = b - a
        return 0.0 if den == 0 else (x - a) / den
    den = c - b
    return 0.0 if den == 0 else (c - x) / den


def gaussmf(x: float, center: float, sigma: float) -> float:
    if sigma <= 0:
        return 0.0
    z = (x - center) / sigma
    return math.exp(-0.5 * z * z)


def left_shoulder(x: float, full_until: float, zero_at: float) -> float:
    if x <= full_until:
        return 1.0
    if x >= zero_at:
        return 0.0
    den = zero_at - full_until
    return 0.0 if den == 0 else (zero_at - x) / den


def right_shoulder(x: float, zero_until: float, full_at: float) -> float:
    if x <= zero_until:
        return 0.0
    if x >= full_at:
        return 1.0
    den = full_at - zero_until
    return 0.0 if den == 0 else (x - zero_until) / den


@dataclass
class FuzzyRule:
    rpm_term: str
    current_term: str
    speed_term: str
    weight: float


class PtoFuzzySpeedController:
    def __init__(
        self,
        rpm_stop: float,
        rpm_full_speed: float,
        min_scale_factor: float,
        no_rule_fallback_speed: float = 100.0,
        use_current_derate_curve: bool = False,
        current_derate_amps: Optional[Iterable[float]] = None,
        current_derate_factors: Optional[Iterable[float]] = None,
    ) -> None:
        self.rpm_stop = rpm_stop
        self.rpm_full_speed = max(rpm_full_speed, rpm_stop + 1.0)
        self.min_scale_factor = clamp(min_scale_factor, 0.0, 1.0)
        self.no_rule_fallback_speed = clamp(no_rule_fallback_speed, 0.0, 100.0)
        self.use_current_derate_curve = bool(use_current_derate_curve)
        self.current_derate_amps = [
            float(value)
            for value in (current_derate_amps or DEFAULT_CURRENT_DERATE_AMPS)
        ]
        self.current_derate_factors = [
            clamp(float(value), 0.0, 1.0)
            for value in (current_derate_factors or DEFAULT_CURRENT_DERATE_FACTORS)
        ]
        self.current_range = (30.0, 310.0)
        self.speed_universe = [float(v) for v in range(0, 101)]
        self.sigma = 8.496

        self.rules: List[FuzzyRule] = [
            FuzzyRule("vrlo_veliki", "vrlo_mala", "veoma_velika", 1.0),
            FuzzyRule("vrlo_veliki", "mala", "velika", 1.0),
            FuzzyRule("vrlo_veliki", "srednja", "srednja", 1.0),
            FuzzyRule("vrlo_veliki", "visoka", "mala", 1.0),
            FuzzyRule("vrlo_veliki", "veoma_visoka", "veoma_mala", 1.0),
            FuzzyRule("vrlo_mali", "veoma_visoka", "stoj", 1.0),
            FuzzyRule("vrlo_mali", "visoka", "veoma_mala", 0.5),
            FuzzyRule("vrlo_mali", "srednja", "mala", 1.0),
            FuzzyRule("vrlo_mali", "mala", "srednja", 1.0),
            FuzzyRule("vrlo_mali", "vrlo_mala", "velika", 0.5),
            FuzzyRule("mali", "veoma_visoka", "veoma_mala", 1.0),
            FuzzyRule("mali", "visoka", "veoma_mala", 0.5),
            FuzzyRule("mali", "srednja", "mala", 1.0),
            FuzzyRule("srednji", "veoma_visoka", "mala", 1.0),
            FuzzyRule("srednji", "visoka", "srednja", 0.5),
            FuzzyRule("veliki", "veoma_visoka", "srednja", 1.0),
            FuzzyRule("veliki", "visoka", "velika", 0.5),
            FuzzyRule("veliki", "vrlo_mala", "veoma_velika", 0.8),
            FuzzyRule("veliki", "mala", "velika", 1.0),
        ]

    def _rpm_membership(self, x: float) -> Dict[str, float]:
        span = self.rpm_full_speed - self.rpm_stop
        p0 = self.rpm_stop
        p1 = self.rpm_stop + span * 0.25
        p2 = self.rpm_stop + span * 0.50
        p3 = self.rpm_stop + span * 0.75
        p4 = self.rpm_full_speed

        return {
            "vrlo_mali": left_shoulder(x, p0, p1),
            "mali": trimf(x, p0, p1, p2),
            "srednji": trimf(x, p1, p2, p3),
            "veliki": trimf(x, p2, p3, p4),
            "vrlo_veliki": right_shoulder(x, p3, p4),
        }

    def _current_membership(self, x: float) -> Dict[str, float]:
        return {
            "vrlo_mala": left_shoulder(x, 80.0, 125.0),
            "mala": trimf(x, 80.0, 125.0, 170.0),
            "srednja": trimf(x, 125.0, 170.0, 215.0),
            "visoka": trimf(x, 170.0, 215.0, 260.0),
            "veoma_visoka": right_shoulder(x, 215.0, 260.0),
        }

    def _speed_membership(self, term: str, x: float) -> float:
        if term == "stoj":
            return trimf(x, 0.0, 0.0, 15.0)
        if term == "veoma_mala":
            return gaussmf(x, 20.0, self.sigma)
        if term == "mala":
            return gaussmf(x, 40.0, self.sigma)
        if term == "srednja":
            return gaussmf(x, 60.0, self.sigma)
        if term == "velika":
            return gaussmf(x, 80.0, self.sigma)
        if term == "veoma_velika":
            return gaussmf(x, 100.0, self.sigma)
        return 0.0

    def _rpm_envelope_factor(self, rpm: float) -> float:
        if rpm <= self.rpm_stop:
            return 0.0
        if rpm >= self.rpm_full_speed:
            return 1.0

        span = self.rpm_full_speed - self.rpm_stop
        ratio = (rpm - self.rpm_stop) / span
        return clamp(
            self.min_scale_factor + ratio * (1.0 - self.min_scale_factor),
            0.0,
            1.0,
        )

    def current_derate_factor(self, current: float) -> float:
        amps = self.current_derate_amps
        factors = self.current_derate_factors

        if current <= amps[0]:
            return factors[0]

        for index in range(1, len(amps)):
            upper_amp = amps[index]
            if current <= upper_amp:
                lower_amp = amps[index - 1]
                lower_factor = factors[index - 1]
                upper_factor = factors[index]
                span = upper_amp - lower_amp
                if span <= 0.0:
                    return min(lower_factor, upper_factor)
                ratio = (current - lower_amp) / span
                return clamp(
                    lower_factor + ratio * (upper_factor - lower_factor),
                    0.0,
                    1.0,
                )

        return factors[-1]

    def evaluate(self, rpm: float, current: float) -> Tuple[float, float]:
        rpm_factor = self._rpm_envelope_factor(rpm)

        if self.use_current_derate_curve:
            current_factor = self.current_derate_factor(current)
            factor = min(current_factor, rpm_factor)
            return factor, factor * 100.0

        rpm_c = clamp(rpm, self.rpm_stop, self.rpm_full_speed)
        current_c = clamp(current, self.current_range[0], self.current_range[1])

        rpm_mu = self._rpm_membership(rpm_c)
        current_mu = self._current_membership(current_c)

        output_activation = {
            "stoj": 0.0,
            "veoma_mala": 0.0,
            "mala": 0.0,
            "srednja": 0.0,
            "velika": 0.0,
            "veoma_velika": 0.0,
        }

        for rule in self.rules:
            alpha = min(rpm_mu[rule.rpm_term], current_mu[rule.current_term]) * rule.weight
            if alpha > output_activation[rule.speed_term]:
                output_activation[rule.speed_term] = alpha

        numerator = 0.0
        denominator = 0.0

        for x in self.speed_universe:
            agg = 0.0
            for term, activation in output_activation.items():
                if activation <= 0.0:
                    continue
                term_mu = self._speed_membership(term, x)
                clipped = min(activation, term_mu)
                if clipped > agg:
                    agg = clipped

            numerator += x * agg
            denominator += agg

        fuzzy_speed = self.no_rule_fallback_speed if denominator <= 1e-9 else (numerator / denominator)
        fuzzy_factor = clamp(fuzzy_speed / 100.0, 0.0, 1.0)
        factor = min(fuzzy_factor, rpm_factor)
        return factor, factor * 100.0


class FuzzyMiddleware(Node):
    def __init__(self) -> None:
        super().__init__("fuzzy_middleware")

        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("pto_rpm_topic", "/pto/rpm")
        self.declare_parameter("pto_current_topic", "/pto/current")
        self.declare_parameter("pto_status_topic", "/extra_motor_status")
        self.declare_parameter("pto_command_topic", "")
        self.declare_parameter("factor_topic", "/fuzzy_debug/factor")
        self.declare_parameter("notification_topic", "/agar_notifications")
        self.declare_parameter("brake_topic", "/brake_status")

        self.declare_parameter("enable_fuzzy", True)
        self.declare_parameter("rpm_full_speed", 2250.0)
        self.declare_parameter("rpm_stop", 1800.0)
        self.declare_parameter("rpm_shutdown", 800.0)
        self.declare_parameter("rpm_shutdown_timeout_sec", 5.0)
        self.declare_parameter("enable_pto_auto_shutdown", False)
        self.declare_parameter("min_scale_factor", 0.0)
        self.declare_parameter("no_rule_fallback_speed", 100.0)
        self.declare_parameter("use_current_derate_curve", False)
        self.declare_parameter("current_derate_amps", DEFAULT_CURRENT_DERATE_AMPS)
        self.declare_parameter("current_derate_factors", DEFAULT_CURRENT_DERATE_FACTORS)
        self.declare_parameter("current_hard_stop", 255.0)
        self.declare_parameter("pto_timeout_sec", 1.5)
        self.declare_parameter("pto_startup_grace_sec", 4.0)
        self.declare_parameter("pto_max_valid_current", 1000.0)
        self.declare_parameter("pto_max_valid_rpm", 4000.0)
        self.declare_parameter("current_hard_stop_samples", 3)
        self.declare_parameter("pass_through_if_pto_missing", False)
        self.declare_parameter("infer_pto_active_from_telemetry", True)
        self.declare_parameter("pto_status_timeout_sec", 2.0)
        self.declare_parameter("pto_infer_active_min_rpm", 50.0)
        self.declare_parameter("pto_infer_active_min_current", 5.0)
        self.declare_parameter("factor_slew_rate_per_sec", 2.0)
        self.declare_parameter("publish_brake_on_stop", False)
        self.declare_parameter("emergency_stop_hold_sec", 3.0)
        self.declare_parameter("resume_last_cmd_after_stop", True)
        self.declare_parameter("notification_cooldown_sec", 10.0)
        self.declare_parameter("notify_invalid_telemetry", False)

        input_cmd_topic = self.get_parameter("input_cmd_topic").value
        output_cmd_topic = self.get_parameter("output_cmd_topic").value
        pto_rpm_topic = self.get_parameter("pto_rpm_topic").value
        pto_current_topic = self.get_parameter("pto_current_topic").value
        pto_status_topic = self.get_parameter("pto_status_topic").value
        pto_command_topic = str(self.get_parameter("pto_command_topic").value or "").strip()
        factor_topic = self.get_parameter("factor_topic").value
        notification_topic = self.get_parameter("notification_topic").value
        brake_topic = self.get_parameter("brake_topic").value

        self.enable_fuzzy = bool(self.get_parameter("enable_fuzzy").value)
        self.rpm_full_speed = float(self.get_parameter("rpm_full_speed").value)
        self.rpm_stop = float(self.get_parameter("rpm_stop").value)
        self.rpm_shutdown = float(self.get_parameter("rpm_shutdown").value)
        self.rpm_shutdown_timeout_sec = float(self.get_parameter("rpm_shutdown_timeout_sec").value)
        self.enable_pto_auto_shutdown = bool(self.get_parameter("enable_pto_auto_shutdown").value)
        self.min_scale_factor = float(self.get_parameter("min_scale_factor").value)
        self.no_rule_fallback_speed = float(self.get_parameter("no_rule_fallback_speed").value)
        self.use_current_derate_curve = bool(self.get_parameter("use_current_derate_curve").value)
        self.current_derate_amps, self.current_derate_factors = self._load_current_derate_curve()
        self.current_hard_stop = float(self.get_parameter("current_hard_stop").value)
        self.pto_timeout_sec = float(self.get_parameter("pto_timeout_sec").value)
        self.pto_startup_grace_sec = float(self.get_parameter("pto_startup_grace_sec").value)
        self.pto_max_valid_current = float(self.get_parameter("pto_max_valid_current").value)
        self.pto_max_valid_rpm = float(self.get_parameter("pto_max_valid_rpm").value)
        self.current_hard_stop_samples = max(1, int(self.get_parameter("current_hard_stop_samples").value))
        self.pass_through_if_pto_missing = bool(self.get_parameter("pass_through_if_pto_missing").value)
        self.infer_pto_active_from_telemetry = bool(
            self.get_parameter("infer_pto_active_from_telemetry").value
        )
        self.pto_status_timeout_sec = float(self.get_parameter("pto_status_timeout_sec").value)
        self.pto_infer_active_min_rpm = float(self.get_parameter("pto_infer_active_min_rpm").value)
        self.pto_infer_active_min_current = float(
            self.get_parameter("pto_infer_active_min_current").value
        )
        self.factor_slew_rate_per_sec = float(self.get_parameter("factor_slew_rate_per_sec").value)
        self.publish_brake_on_stop = bool(self.get_parameter("publish_brake_on_stop").value)
        self.emergency_stop_hold_sec = float(self.get_parameter("emergency_stop_hold_sec").value)
        self.resume_last_cmd_after_stop = bool(
            self.get_parameter("resume_last_cmd_after_stop").value
        )
        self.notification_cooldown_sec = float(self.get_parameter("notification_cooldown_sec").value)
        self.notify_invalid_telemetry = bool(self.get_parameter("notify_invalid_telemetry").value)

        self.pto_command_topic = pto_command_topic
        self.cmd_vel_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self.pto_command_pub = (
            self.create_publisher(UInt64, pto_command_topic, 10)
            if pto_command_topic
            else None
        )
        self.factor_pub = self.create_publisher(Float32, factor_topic, 10)
        self.noty_pub = self.create_publisher(String, notification_topic, 10)
        self.brake_pub = self.create_publisher(Int16, brake_topic, 10)

        self.create_subscription(Float32, pto_rpm_topic, self._on_pto_rpm, 10)
        self.create_subscription(Float32, pto_current_topic, self._on_pto_current, 10)
        self.create_subscription(UInt64, pto_status_topic, self._on_pto_status, 10)
        self.create_subscription(Twist, input_cmd_topic, self._on_cmd_vel_raw, 10)
        self.create_timer(0.2, self._watch_pto_shutdown)
        self.create_timer(0.1, self._watch_pto_current_hard_stop)
        self.create_timer(0.1, self._maintain_emergency_stop)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.controller = PtoFuzzySpeedController(
            rpm_full_speed=self.rpm_full_speed,
            rpm_stop=self.rpm_stop,
            min_scale_factor=self.min_scale_factor,
            no_rule_fallback_speed=self.no_rule_fallback_speed,
            use_current_derate_curve=self.use_current_derate_curve,
            current_derate_amps=self.current_derate_amps,
            current_derate_factors=self.current_derate_factors,
        )

        self.current_rpm = 0.0
        self.current_current = 0.0
        self.last_rpm_ts = 0.0
        self.last_current_ts = 0.0
        self.last_pto_ts = 0.0
        self.pto_active = False
        self.effective_pto_active = False
        self.last_pto_status_ts = 0.0
        self.pto_active_since = 0.0
        self.rpm_shutdown_start_ts = 0.0
        self.pto_shutdown_commanded = False
        self.current_hard_stop_count = 0

        self.last_factor = 1.0
        self.last_factor_ts = time.monotonic()
        self.last_noty_ts = 0.0
        self.last_invalid_telemetry_noty_ts = 0.0
        self.emergency_stop_until_ts = 0.0
        self.last_raw_cmd: Optional[Twist] = None
        self.resume_last_cmd_pending = False

        self.get_logger().info("fuzzy_middleware started")
        self.get_logger().info("enable_fuzzy=%s" % self.enable_fuzzy)
        self.get_logger().info(
            "topics: %s -> %s | pto: (%s, %s, %s) | pto command: %s"
            % (
                input_cmd_topic,
                output_cmd_topic,
                pto_rpm_topic,
                pto_current_topic,
                pto_status_topic,
                pto_command_topic or "disabled",
            )
        )
        self.get_logger().info(
            "fuzzy raw PTO RPM control: full=%.1f stop=%.1f shutdown=%.1f for %.1fs"
            % (
                self.rpm_full_speed,
                self.rpm_stop,
                self.rpm_shutdown,
                self.rpm_shutdown_timeout_sec,
            )
        )
        self.get_logger().info(
            "current derate curve: enabled=%s amps=%s factors=%s hard_stop=%.1fA"
            % (
                self.use_current_derate_curve,
                self.current_derate_amps,
                self.current_derate_factors,
                self.current_hard_stop,
            )
        )

    def _on_parameter_update(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "enable_fuzzy":
                self.enable_fuzzy = bool(param.value)
                self._reset_hard_stop_counts()
                self.rpm_shutdown_start_ts = 0.0
                self.pto_shutdown_commanded = False
                self.last_factor = 1.0
                self.last_factor_ts = time.monotonic()
                state = "enabled" if self.enable_fuzzy else "disabled/pass-through"
                self.get_logger().warn("Fuzzy protection %s" % state)
            elif param.name == "publish_brake_on_stop":
                self.publish_brake_on_stop = bool(param.value)
                self.get_logger().warn(
                    "Fuzzy brake-on-stop %s"
                    % ("enabled" if self.publish_brake_on_stop else "disabled")
                )
            elif param.name == "current_hard_stop":
                self.current_hard_stop = float(param.value)
                self._reset_hard_stop_counts()
                self.get_logger().warn(
                    "Fuzzy current hard stop set to %.1fA" % self.current_hard_stop
                )
            elif param.name == "current_hard_stop_samples":
                self.current_hard_stop_samples = max(1, int(param.value))
                self._reset_hard_stop_counts()
                self.get_logger().warn(
                    "Fuzzy current hard stop samples set to %d"
                    % self.current_hard_stop_samples
                )
            elif param.name == "emergency_stop_hold_sec":
                self.emergency_stop_hold_sec = max(0.0, float(param.value))
                self.get_logger().warn(
                    "Fuzzy emergency stop hold set to %.1fs"
                    % self.emergency_stop_hold_sec
                )
            elif param.name == "resume_last_cmd_after_stop":
                self.resume_last_cmd_after_stop = bool(param.value)
                if not self.resume_last_cmd_after_stop:
                    self.resume_last_cmd_pending = False
                self.get_logger().warn(
                    "Fuzzy last command resume after stop %s"
                    % ("enabled" if self.resume_last_cmd_after_stop else "disabled")
                )
            elif param.name == "infer_pto_active_from_telemetry":
                self.infer_pto_active_from_telemetry = bool(param.value)
                self.get_logger().warn(
                    "Fuzzy PTO active telemetry inference %s"
                    % ("enabled" if self.infer_pto_active_from_telemetry else "disabled")
                )
            elif param.name == "pto_status_timeout_sec":
                self.pto_status_timeout_sec = max(0.0, float(param.value))
                self.get_logger().warn(
                    "Fuzzy PTO status timeout set to %.1fs" % self.pto_status_timeout_sec
                )
            elif param.name == "pto_infer_active_min_rpm":
                self.pto_infer_active_min_rpm = max(0.0, float(param.value))
                self.get_logger().warn(
                    "Fuzzy PTO active RPM threshold set to %.1f"
                    % self.pto_infer_active_min_rpm
                )
            elif param.name == "pto_infer_active_min_current":
                self.pto_infer_active_min_current = max(0.0, float(param.value))
                self.get_logger().warn(
                    "Fuzzy PTO active current threshold set to %.1fA"
                    % self.pto_infer_active_min_current
                )
        return SetParametersResult(successful=True)

    def _load_current_derate_curve(self) -> Tuple[List[float], List[float]]:
        amps = self._float_list_parameter("current_derate_amps", DEFAULT_CURRENT_DERATE_AMPS)
        factors = self._float_list_parameter("current_derate_factors", DEFAULT_CURRENT_DERATE_FACTORS)

        if len(amps) != len(factors) or len(amps) < 2:
            self.get_logger().warn(
                "Invalid current derate curve length; using defaults amps=%s factors=%s"
                % (DEFAULT_CURRENT_DERATE_AMPS, DEFAULT_CURRENT_DERATE_FACTORS)
            )
            return list(DEFAULT_CURRENT_DERATE_AMPS), list(DEFAULT_CURRENT_DERATE_FACTORS)

        for index in range(1, len(amps)):
            if amps[index] <= amps[index - 1]:
                self.get_logger().warn(
                    "current_derate_amps must be strictly increasing; using defaults"
                )
                return list(DEFAULT_CURRENT_DERATE_AMPS), list(DEFAULT_CURRENT_DERATE_FACTORS)

        return amps, [clamp(value, 0.0, 1.0) for value in factors]

    def _float_list_parameter(self, name: str, fallback: List[float]) -> List[float]:
        value = self.get_parameter(name).value
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            self.get_logger().warn("Invalid %s parameter; using default %s" % (name, fallback))
            return list(fallback)

    def _on_pto_rpm(self, msg: Float32) -> None:
        value = float(msg.data)
        if not self._is_valid_pto_value(value, self.pto_max_valid_rpm):
            self._warn_invalid_telemetry("rpm", value, self.pto_max_valid_rpm)
            return
        now = time.monotonic()
        self.current_rpm = value
        self.last_rpm_ts = now
        self._mark_pto_telemetry_received(now)
        self._refresh_pto_active_state(now)

    def _on_pto_current(self, msg: Float32) -> None:
        value = float(msg.data)
        if not self._is_valid_pto_value(value, self.pto_max_valid_current):
            self._warn_invalid_telemetry("current", value, self.pto_max_valid_current)
            return
        now = time.monotonic()
        self.current_current = value
        self.last_current_ts = now
        self._refresh_pto_active_state(now)
        if self.effective_pto_active and value >= self.current_hard_stop:
            self.current_hard_stop_count = min(
                self.current_hard_stop_count + 1,
                self.current_hard_stop_samples,
            )
        else:
            self.current_hard_stop_count = 0
        self._mark_pto_telemetry_received(now)

    def _on_pto_status(self, msg: UInt64) -> None:
        now = time.monotonic()
        self.pto_active = int(msg.data) > 0
        self.last_pto_status_ts = now
        self._set_effective_pto_active(self.pto_active, now)

    def _set_effective_pto_active(self, active: bool, now: float) -> None:
        was_active = self.effective_pto_active
        self.effective_pto_active = bool(active)

        if self.effective_pto_active and not was_active:
            self.pto_active_since = now
            self.last_factor_ts = now
            self.emergency_stop_until_ts = 0.0
            self.rpm_shutdown_start_ts = 0.0
            self.pto_shutdown_commanded = False
            self._reset_hard_stop_counts()
            if self.enable_fuzzy:
                self.get_logger().info("PTO implement active: fuzzy protection enabled")
            else:
                self.get_logger().info("PTO implement active: fuzzy protection bypassed")
        elif not self.effective_pto_active and was_active:
            self.last_factor = 1.0
            self.last_factor_ts = time.monotonic()
            self.pto_active_since = 0.0
            self.emergency_stop_until_ts = 0.0
            self.rpm_shutdown_start_ts = 0.0
            self.pto_shutdown_commanded = False
            self._reset_hard_stop_counts()
            self.get_logger().info("PTO implement inactive: fuzzy protection bypassed")

    def _refresh_pto_active_state(self, now: float) -> bool:
        status_recent = (
            self.last_pto_status_ts > 0.0
            and (now - self.last_pto_status_ts) <= self.pto_status_timeout_sec
        )

        if status_recent:
            self._set_effective_pto_active(self.pto_active, now)
            return self.effective_pto_active

        if not self.infer_pto_active_from_telemetry:
            self._set_effective_pto_active(self.pto_active, now)
            return self.effective_pto_active

        rpm_age, current_age = self._pto_telemetry_ages(now)
        inferred_active = (
            (rpm_age <= self.pto_timeout_sec and self.current_rpm >= self.pto_infer_active_min_rpm)
            or (
                current_age <= self.pto_timeout_sec
                and self.current_current >= self.pto_infer_active_min_current
            )
        )
        self._set_effective_pto_active(inferred_active, now)
        return self.effective_pto_active

    def _mark_pto_telemetry_received(self, now: float) -> None:
        self.last_pto_ts = now

    def _is_valid_pto_value(self, value: float, max_value: float) -> bool:
        return math.isfinite(value) and 0.0 <= value <= max_value

    def _warn_invalid_telemetry(self, name: str, value: float, max_value: float) -> None:
        now = time.monotonic()
        if (now - self.last_invalid_telemetry_noty_ts) < self.notification_cooldown_sec:
            return
        self.last_invalid_telemetry_noty_ts = now
        reason = "Ignoring invalid PTO %s telemetry: %.1f outside 0..%.1f" % (
            name,
            value,
            max_value,
        )
        self.get_logger().warn(reason)
        if self.notify_invalid_telemetry:
            msg = String()
            msg.data = reason
            self.noty_pub.publish(msg)

    def _pto_telemetry_ages(self, now: float) -> Tuple[float, float]:
        rpm_age = float("inf") if self.last_rpm_ts <= 0.0 else now - self.last_rpm_ts
        current_age = float("inf") if self.last_current_ts <= 0.0 else now - self.last_current_ts
        return rpm_age, current_age

    def _reset_hard_stop_counts(self) -> None:
        self.current_hard_stop_count = 0

    def _on_cmd_vel_raw(self, msg: Twist) -> None:
        self.last_raw_cmd = self._copy_twist(msg)
        self._process_cmd_vel_raw(msg)

    def _process_cmd_vel_raw(self, msg: Twist) -> None:
        now = time.monotonic()

        if not self.enable_fuzzy:
            self._reset_hard_stop_counts()
            self.rpm_shutdown_start_ts = 0.0
            self.emergency_stop_until_ts = 0.0
            self.resume_last_cmd_pending = False
            self._publish_passthrough_cmd(msg, now)
            return

        if self._emergency_stop_active(now):
            self._publish_emergency_stop_command()
            return

        rpm_age, current_age = self._pto_telemetry_ages(now)
        stale_pto = rpm_age > self.pto_timeout_sec or current_age > self.pto_timeout_sec

        if not self._refresh_pto_active_state(now):
            self._reset_hard_stop_counts()
            self._publish_cmd_with_factor(msg, 1.0, now)
            return

        in_startup_grace = (
            self.pto_active_since > 0.0
            and (now - self.pto_active_since) < self.pto_startup_grace_sec
        )

        startup_not_ready = (
            stale_pto
            or self.current_current >= self.current_hard_stop
            or self.current_rpm < self.rpm_stop
        )
        if in_startup_grace and startup_not_ready:
            self._reset_hard_stop_counts()
            self._publish_cmd_with_factor(msg, 0.0, now)
            return

        if stale_pto:
            if self.pass_through_if_pto_missing:
                self._publish_cmd_with_factor(msg, 1.0, now)
            else:
                self._emergency_stop(
                    "PTO telemetry timeout while PTO is active: rpm_age=%.2fs current_age=%.2fs"
                    % (rpm_age, current_age)
                )
            return

        if self.current_hard_stop_count > 0:
            if self.current_hard_stop_count >= self.current_hard_stop_samples:
                self._emergency_stop(
                    reason=(
                        "PTO hard stop: current %.1fA >= %.1fA (%d/%d samples)"
                        % (
                            self.current_current,
                            self.current_hard_stop,
                            self.current_hard_stop_count,
                            self.current_hard_stop_samples,
                        )
                    )
                )
                return
            self._publish_cmd_with_factor(msg, 0.0, now)
            return

        factor, _speed_out = self.controller.evaluate(self.current_rpm, self.current_current)

        if factor <= 0.0:
            if (
                self.use_current_derate_curve
                and self.controller.current_derate_factor(self.current_current) <= 0.0
            ):
                self._emergency_stop(
                    reason=(
                        "PTO current stop: current=%.1fA reached current derate stop"
                        % self.current_current
                    )
                )
                return
            self._emergency_stop(
                reason=(
                    "PTO rpm stop: rpm=%.1f below %.1f"
                    % (self.current_rpm, self.rpm_stop)
                )
            )
            return

        self._publish_cmd_with_factor(msg, factor, now)

    def _watch_pto_current_hard_stop(self) -> None:
        now = time.monotonic()
        if not self.enable_fuzzy or not self._refresh_pto_active_state(now):
            return
        if self._emergency_stop_active(now):
            return

        _rpm_age, current_age = self._pto_telemetry_ages(now)
        if current_age > self.pto_timeout_sec:
            return

        if self.current_hard_stop_count >= self.current_hard_stop_samples:
            self._emergency_stop(
                reason=(
                    "PTO hard stop: current %.1fA >= %.1fA (%d/%d samples)"
                    % (
                        self.current_current,
                        self.current_hard_stop,
                        self.current_hard_stop_count,
                        self.current_hard_stop_samples,
                    )
                )
            )

    def _emergency_stop_active(self, now: float) -> bool:
        return self.emergency_stop_until_ts > 0.0 and now < self.emergency_stop_until_ts

    def _maintain_emergency_stop(self) -> None:
        if self._emergency_stop_active(time.monotonic()):
            self._publish_emergency_stop_command()
            return
        self._resume_last_cmd_after_stop()

    def _resume_last_cmd_after_stop(self) -> None:
        if (
            not self.resume_last_cmd_after_stop
            or not self.resume_last_cmd_pending
            or self.last_raw_cmd is None
        ):
            return

        raw = self._copy_twist(self.last_raw_cmd)
        self.resume_last_cmd_pending = False
        self._process_cmd_vel_raw(raw)

    def _publish_emergency_stop_command(self) -> None:
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        self._publish_factor(0.0)

        if self.publish_brake_on_stop:
            brake = Int16()
            brake.data = 1
            self.brake_pub.publish(brake)

    def _watch_pto_shutdown(self) -> None:
        now = time.monotonic()
        if (
            not self.enable_fuzzy
            or not self.enable_pto_auto_shutdown
            or not self._refresh_pto_active_state(now)
        ):
            self.rpm_shutdown_start_ts = 0.0
            return

        rpm_age, _current_age = self._pto_telemetry_ages(now)
        if rpm_age > self.pto_timeout_sec:
            self.rpm_shutdown_start_ts = 0.0
            return

        if self.current_rpm >= self.rpm_shutdown:
            self.rpm_shutdown_start_ts = 0.0
            return

        if self.rpm_shutdown_start_ts <= 0.0:
            self.rpm_shutdown_start_ts = now
            return

        elapsed = now - self.rpm_shutdown_start_ts
        if elapsed < self.rpm_shutdown_timeout_sec:
            return

        reason = (
            "PTO auto shutdown: rpm %.1f below %.1f for %.1fs"
            % (self.current_rpm, self.rpm_shutdown, elapsed)
        )
        self._command_pto_off(reason)
        self._emergency_stop(reason)

    def _command_pto_off(self, reason: str) -> None:
        if self.pto_shutdown_commanded:
            return
        if self.pto_command_pub is None:
            self.get_logger().warn("PTO auto shutdown command topic disabled: %s" % reason)
            return

        self.pto_shutdown_commanded = True
        msg = UInt64()
        msg.data = 0
        for _ in range(3):
            self.pto_command_pub.publish(msg)
        self.get_logger().warn("Commanded PTO off: %s" % reason)

    def _apply_slew_limit(self, factor: float, now: float) -> float:
        dt = max(0.0, now - self.last_factor_ts)
        max_delta = self.factor_slew_rate_per_sec * dt
        limited = clamp(factor, self.last_factor - max_delta, self.last_factor + max_delta)
        self.last_factor = limited
        self.last_factor_ts = now
        return limited

    def _publish_cmd_with_factor(self, raw: Twist, factor: float, now: float) -> None:
        factor_limited = self._apply_slew_limit(factor, now)

        self._publish_scaled_cmd(raw, factor_limited)

    def _publish_passthrough_cmd(self, raw: Twist, now: float) -> None:
        self.last_factor = 1.0
        self.last_factor_ts = now
        self._publish_scaled_cmd(raw, 1.0)

    def _publish_scaled_cmd(self, raw: Twist, factor: float) -> None:
        out = Twist()
        out.linear.x = raw.linear.x * factor
        out.linear.y = raw.linear.y
        out.linear.z = raw.linear.z
        out.angular.x = raw.angular.x
        out.angular.y = raw.angular.y
        out.angular.z = raw.angular.z

        self.cmd_vel_pub.publish(out)
        self._publish_factor(factor)

    def _copy_twist(self, raw: Twist) -> Twist:
        copied = Twist()
        copied.linear.x = raw.linear.x
        copied.linear.y = raw.linear.y
        copied.linear.z = raw.linear.z
        copied.angular.x = raw.angular.x
        copied.angular.y = raw.angular.y
        copied.angular.z = raw.angular.z
        return copied

    def _publish_factor(self, factor: float) -> None:
        msg = Float32()
        msg.data = float(factor)
        self.factor_pub.publish(msg)

    def _emergency_stop(self, reason: str) -> None:
        now = time.monotonic()
        self.last_factor = 0.0
        self.last_factor_ts = now
        self.emergency_stop_until_ts = max(
            self.emergency_stop_until_ts,
            now + self.emergency_stop_hold_sec,
        )
        self.resume_last_cmd_pending = (
            self.resume_last_cmd_after_stop and self.last_raw_cmd is not None
        )
        self._publish_emergency_stop_command()

        if (now - self.last_noty_ts) >= self.notification_cooldown_sec:
            self.last_noty_ts = now
            self.get_logger().warn(reason)
            msg = String()
            msg.data = reason
            self.noty_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FuzzyMiddleware()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
