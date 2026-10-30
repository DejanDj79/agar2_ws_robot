#!/usr/bin/env python3

import re
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.srv import ReportFault


@dataclass
class FaultState:
    active: bool = False
    bad_count: int = 0
    good_count: int = 0
    last_level: int = 0
    last_report_time: float = 0.0


class AgarMedkitDiagnosticBridge(Node):
    def __init__(self):
        super().__init__("agar_medkit_diagnostic_bridge")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("fault_manager_service", "/fault_manager/report_fault")
        self.declare_parameter("startup_grace_sec", 20.0)
        self.declare_parameter("failure_threshold", 3)
        self.declare_parameter("recovery_threshold", 3)
        self.declare_parameter("active_repeat_sec", 300.0)
        self.declare_parameter(
            "diagnostic_names",
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter(
            "fault_codes",
            Parameter.Type.STRING_ARRAY,
        )

        diagnostic_names = list(self.get_parameter("diagnostic_names").value)
        fault_codes = list(self.get_parameter("fault_codes").value)
        if len(diagnostic_names) != len(fault_codes):
            raise RuntimeError("diagnostic_names and fault_codes must have equal lengths")

        self.mapping = {
            str(name): self._normalize_fault_code(code)
            for name, code in zip(diagnostic_names, fault_codes)
            if str(name).strip() and str(code).strip()
        }
        if not self.mapping:
            raise RuntimeError("At least one diagnostic mapping is required")

        self.startup_grace_sec = max(
            0.0,
            float(self.get_parameter("startup_grace_sec").value),
        )
        self.failure_threshold = max(
            1,
            int(self.get_parameter("failure_threshold").value),
        )
        self.recovery_threshold = max(
            1,
            int(self.get_parameter("recovery_threshold").value),
        )
        self.active_repeat_sec = max(
            0.0,
            float(self.get_parameter("active_repeat_sec").value),
        )
        self.started_at = time.monotonic()
        self.last_service_warning_time = 0.0
        self.states = {
            name: FaultState()
            for name in self.mapping
        }

        service_name = str(self.get_parameter("fault_manager_service").value)
        self.client = self.create_client(ReportFault, service_name)

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.subscription = self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self._diagnostics_callback,
            qos,
        )

        self.get_logger().info(
            f"AgAR Medkit diagnostic bridge ready: topic={diagnostics_topic} "
            f"mappings={len(self.mapping)} "
            f"failure_threshold={self.failure_threshold} "
            f"recovery_threshold={self.recovery_threshold}"
        )

    @staticmethod
    def _normalize_fault_code(value):
        code = re.sub(r"[^A-Z0-9]+", "_", str(value).upper()).strip("_")
        if not code:
            raise ValueError("Fault code cannot be empty")
        return code[:64]

    @staticmethod
    def _level_number(value):
        if isinstance(value, (bytes, bytearray)):
            return value[0] if value else 0
        return int(value)

    def _diagnostics_callback(self, msg):
        in_grace_period = (time.monotonic() - self.started_at) < self.startup_grace_sec

        for status in msg.status:
            if status.name not in self.mapping:
                continue

            state = self.states[status.name]
            level = self._level_number(status.level)
            if in_grace_period:
                state.bad_count = 0
                state.good_count = 0
                continue

            if level == 0:
                self._handle_ok(status.name, state)
            else:
                self._handle_fault(status, level, state)

    def _handle_ok(self, diagnostic_name, state):
        state.bad_count = 0
        state.good_count += 1
        if not state.active or state.good_count < self.recovery_threshold:
            return

        if self._send_report(
            self.mapping[diagnostic_name],
            ReportFault.Request.EVENT_PASSED,
            0,
            "",
        ):
            state.active = False
            state.good_count = 0
            state.last_level = 0
            state.last_report_time = time.monotonic()
            fault_code = self.mapping[diagnostic_name]
            self.get_logger().info(f"Fault recovered: {fault_code}")

    def _handle_fault(self, status, level, state):
        state.good_count = 0
        state.bad_count += 1
        now = time.monotonic()
        first_report = not state.active and state.bad_count >= self.failure_threshold
        severity_escalated = state.active and level > state.last_level
        repeat_due = (
            state.active
            and self.active_repeat_sec > 0.0
            and (now - state.last_report_time) >= self.active_repeat_sec
        )
        if not (first_report or severity_escalated or repeat_due):
            return

        fault_code = self.mapping[status.name]
        severity = self._severity(level)
        description = status.message or status.name
        if self._send_report(
            fault_code,
            ReportFault.Request.EVENT_FAILED,
            severity,
            description,
        ):
            state.active = True
            state.last_level = level
            state.last_report_time = now
            self.get_logger().warn(
                f"Fault reported: {fault_code} level={level} "
                f"diagnostic={status.name}"
            )

    @staticmethod
    def _severity(level):
        if level == 1:
            return 1
        if level == 2:
            return 2
        return 3

    def _send_report(self, fault_code, event_type, severity, description):
        if not self.client.service_is_ready():
            now = time.monotonic()
            if (now - self.last_service_warning_time) >= 10.0:
                self.get_logger().warn(
                    "Fault Manager is unavailable; keeping pending state for "
                    f"{fault_code}"
                )
                self.last_service_warning_time = now
            return False

        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = self.get_fully_qualified_name()
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda result, code=fault_code: self._report_result(code, result)
        )
        return True

    def _report_result(self, fault_code, future):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"Fault report failed for {fault_code}: {ex}")
            return
        if not response.accepted:
            self.get_logger().error(f"Fault Manager rejected {fault_code}")


def main(args=None):
    rclpy.init(args=args)
    node = AgarMedkitDiagnosticBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    rclpy.try_shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == "__main__":
    main()
