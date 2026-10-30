#!/usr/bin/env python3
"""Publish SLAM-specific ROS diagnostics for Medkit fault handling."""

import time
from collections import deque

from agar_slam.slam_health_logic import (
    expected_engine_nodes,
    planar_step,
    sample_rate_hz,
    yaw_from_quaternion,
)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener

OK = 0
WARN = 1
ERROR = 2

DIAGNOSTIC_ENGINE = 'slam/engine'
DIAGNOSTIC_SCAN = 'slam/scan_input'
DIAGNOSTIC_TF = 'slam/tf_chain'
DIAGNOSTIC_MAP = 'slam/map_availability'
DIAGNOSTIC_JUMP = 'slam/localization_jump'
DIAGNOSTIC_TIMING = 'slam/message_timing'


class SlamHealthMonitor(Node):
    """Observe SLAM inputs, outputs, timing, TF, and correction jumps."""

    def __init__(self):
        """Create subscriptions, TF state, timers, and diagnostics output."""
        super().__init__('slam_health_monitor')

        self.declare_parameter('mode', 'off')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('default_scan_frame', 'velodyne')
        self.declare_parameter('startup_grace_sec', 45.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('tf_sample_period_sec', 0.1)
        self.declare_parameter('scan_rate_window_sec', 10.0)
        self.declare_parameter('min_scan_rate_hz', 5.0)
        self.declare_parameter('scan_warn_age_sec', 0.5)
        self.declare_parameter('scan_error_age_sec', 2.0)
        self.declare_parameter('tf_warn_age_sec', 0.5)
        self.declare_parameter('tf_error_age_sec', 2.0)
        self.declare_parameter('scan_tf_check_delay_sec', 0.3)
        self.declare_parameter('timing_window_sec', 10.0)
        self.declare_parameter('timing_warn_failure_ratio', 0.1)
        self.declare_parameter('timing_error_failure_ratio', 0.5)
        self.declare_parameter('jump_warn_translation_m', 0.2)
        self.declare_parameter('jump_error_translation_m', 0.4)
        self.declare_parameter('jump_warn_yaw_deg', 5.0)
        self.declare_parameter('jump_error_yaw_deg', 12.0)
        self.declare_parameter('jump_hold_sec', 10.0)

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_frame = str(
            self.get_parameter('default_scan_frame').value
        )
        self.startup_grace_sec = self._positive_parameter(
            'startup_grace_sec'
        )
        self.scan_rate_window_sec = self._positive_parameter(
            'scan_rate_window_sec'
        )
        self.min_scan_rate_hz = self._positive_parameter(
            'min_scan_rate_hz'
        )
        self.scan_warn_age_sec = self._positive_parameter(
            'scan_warn_age_sec'
        )
        self.scan_error_age_sec = self._positive_parameter(
            'scan_error_age_sec'
        )
        self.tf_warn_age_sec = self._positive_parameter(
            'tf_warn_age_sec'
        )
        self.tf_error_age_sec = self._positive_parameter(
            'tf_error_age_sec'
        )
        self.scan_tf_check_delay_sec = self._positive_parameter(
            'scan_tf_check_delay_sec'
        )
        self.timing_window_sec = self._positive_parameter(
            'timing_window_sec'
        )
        self.timing_warn_failure_ratio = float(
            self.get_parameter('timing_warn_failure_ratio').value
        )
        self.timing_error_failure_ratio = float(
            self.get_parameter('timing_error_failure_ratio').value
        )
        self.jump_warn_translation_m = self._positive_parameter(
            'jump_warn_translation_m'
        )
        self.jump_error_translation_m = self._positive_parameter(
            'jump_error_translation_m'
        )
        self.jump_warn_yaw_deg = self._positive_parameter(
            'jump_warn_yaw_deg'
        )
        self.jump_error_yaw_deg = self._positive_parameter(
            'jump_error_yaw_deg'
        )
        self.jump_hold_sec = self._positive_parameter('jump_hold_sec')

        self.started_at = time.monotonic()
        self.scan_received_times = deque()
        self.pending_scan_tf_checks = deque()
        self.scan_tf_results = deque()
        self.jump_events = deque()
        self.last_scan_received_at = None
        self.last_scan_stamp_ns = None
        self.out_of_order_scan_count = 0
        self.map_received = False
        self.map_width = 0
        self.map_height = 0
        self.previous_map_odom = None
        self.previous_map_odom_stamp_ns = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            diagnostics_qos,
        )

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            scan_qos,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )

        check_period = self._positive_parameter('check_period_sec')
        sample_period = self._positive_parameter('tf_sample_period_sec')
        self.check_timer = self.create_timer(
            check_period,
            self._publish_diagnostics,
        )
        self.sample_timer = self.create_timer(
            sample_period,
            self._sample_tf,
        )

        self.get_logger().info(
            'SLAM health monitor ready: mode=%s scan=%s map=%s'
            % (self.mode, self.scan_topic, self.map_topic)
        )

    def _positive_parameter(self, name):
        return max(0.001, float(self.get_parameter(name).value))

    def _is_enabled(self):
        return self.mode != 'off'

    def _in_startup_grace(self):
        return (time.monotonic() - self.started_at) < self.startup_grace_sec

    def _scan_callback(self, msg):
        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp)
        stamp_ns = stamp.nanoseconds
        if (
            self.last_scan_stamp_ns is not None
            and stamp_ns <= self.last_scan_stamp_ns
        ):
            self.out_of_order_scan_count += 1
        self.last_scan_stamp_ns = stamp_ns
        self.last_scan_received_at = now
        self.scan_received_times.append(now)
        if msg.header.frame_id:
            self.scan_frame = msg.header.frame_id
        self.pending_scan_tf_checks.append(
            (now, stamp, self.scan_frame)
        )

    def _map_callback(self, msg):
        self.map_received = True
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)

    def _sample_tf(self):
        now = time.monotonic()
        self._trim_windows(now)
        self._check_pending_scan_transforms(now)
        if not self._is_enabled() or self._in_startup_grace():
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                Time(),
            )
        except TransformException:
            return

        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        if stamp_ns == self.previous_map_odom_stamp_ns:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        current = (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )
        if (
            self.previous_map_odom is not None
            and self.previous_map_odom_stamp_ns is not None
            and stamp_ns > self.previous_map_odom_stamp_ns
        ):
            translation_step, yaw_step = planar_step(
                self.previous_map_odom,
                current,
            )
            level = self._jump_level(translation_step, yaw_step)
            if level != OK:
                self.jump_events.append(
                    (now, level, translation_step, yaw_step)
                )

        self.previous_map_odom = current
        self.previous_map_odom_stamp_ns = stamp_ns

    def _check_pending_scan_transforms(self, now):
        while self.pending_scan_tf_checks:
            received_at, stamp, frame = self.pending_scan_tf_checks[0]
            if (now - received_at) < self.scan_tf_check_delay_sec:
                break
            self.pending_scan_tf_checks.popleft()
            if not self._is_enabled() or self._in_startup_grace():
                continue
            available = self.tf_buffer.can_transform(
                self.odom_frame,
                frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
            self.scan_tf_results.append((now, bool(available)))

    def _trim_windows(self, now):
        scan_cutoff = now - self.scan_rate_window_sec
        while (
            self.scan_received_times
            and self.scan_received_times[0] < scan_cutoff
        ):
            self.scan_received_times.popleft()

        timing_cutoff = now - self.timing_window_sec
        while (
            self.scan_tf_results
            and self.scan_tf_results[0][0] < timing_cutoff
        ):
            self.scan_tf_results.popleft()

        jump_cutoff = now - self.jump_hold_sec
        while self.jump_events and self.jump_events[0][0] < jump_cutoff:
            self.jump_events.popleft()

    def _jump_level(self, translation_step, yaw_step):
        if (
            translation_step >= self.jump_error_translation_m
            or yaw_step >= self.jump_error_yaw_deg
        ):
            return ERROR
        if (
            translation_step >= self.jump_warn_translation_m
            or yaw_step >= self.jump_warn_yaw_deg
        ):
            return WARN
        return OK

    def _publish_diagnostics(self):
        now = time.monotonic()
        self._trim_windows(now)
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()

        if not self._is_enabled():
            values = {'mode': self.mode}
            message.status = [
                self._status(name, OK, 'SLAM mode is off', values)
                for name in (
                    DIAGNOSTIC_ENGINE,
                    DIAGNOSTIC_SCAN,
                    DIAGNOSTIC_TF,
                    DIAGNOSTIC_MAP,
                    DIAGNOSTIC_JUMP,
                    DIAGNOSTIC_TIMING,
                )
            ]
        elif self._in_startup_grace():
            remaining = max(
                0.0,
                self.startup_grace_sec - (now - self.started_at),
            )
            values = {
                'mode': self.mode,
                'grace_remaining_sec': '%.1f' % remaining,
            }
            message.status = [
                self._status(name, OK, 'SLAM startup grace period', values)
                for name in (
                    DIAGNOSTIC_ENGINE,
                    DIAGNOSTIC_SCAN,
                    DIAGNOSTIC_TF,
                    DIAGNOSTIC_MAP,
                    DIAGNOSTIC_JUMP,
                    DIAGNOSTIC_TIMING,
                )
            ]
        else:
            message.status = [
                self._engine_status(),
                self._scan_status(now),
                self._tf_status(),
                self._map_status(),
                self._jump_status(),
                self._timing_status(),
            ]

        self.publisher.publish(message)

    def _engine_status(self):
        expected = expected_engine_nodes(self.mode)
        available = self._node_names()
        missing = [name for name in expected if name not in available]
        values = {
            'mode': self.mode,
            'expected_nodes': ', '.join(expected),
            'missing_nodes': ', '.join(missing),
        }
        if missing:
            return self._status(
                DIAGNOSTIC_ENGINE,
                ERROR,
                'Missing SLAM/localization node(s): %s'
                % ', '.join(missing),
                values,
            )
        return self._status(
            DIAGNOSTIC_ENGINE,
            OK,
            'Expected SLAM/localization nodes are running',
            values,
        )

    def _scan_status(self, now):
        if self.last_scan_received_at is None:
            return self._status(
                DIAGNOSTIC_SCAN,
                ERROR,
                'No scan messages received on %s' % self.scan_topic,
                {'topic': self.scan_topic, 'frame': self.scan_frame},
            )

        age = now - self.last_scan_received_at
        rate = sample_rate_hz(self.scan_received_times)
        values = {
            'topic': self.scan_topic,
            'frame': self.scan_frame,
            'age_sec': '%.3f' % age,
            'rate_hz': '%.2f' % rate,
            'minimum_rate_hz': '%.2f' % self.min_scan_rate_hz,
            'out_of_order_count': str(self.out_of_order_scan_count),
        }
        if age >= self.scan_error_age_sec:
            return self._status(
                DIAGNOSTIC_SCAN,
                ERROR,
                'Scan input is stale (%.2f s)' % age,
                values,
            )
        if age >= self.scan_warn_age_sec:
            return self._status(
                DIAGNOSTIC_SCAN,
                WARN,
                'Scan input is delayed (%.2f s)' % age,
                values,
            )
        if rate < (self.min_scan_rate_hz * 0.5):
            return self._status(
                DIAGNOSTIC_SCAN,
                ERROR,
                'Scan rate is critically low (%.2f Hz)' % rate,
                values,
            )
        if rate < self.min_scan_rate_hz:
            return self._status(
                DIAGNOSTIC_SCAN,
                WARN,
                'Scan rate is low (%.2f Hz)' % rate,
                values,
            )
        return self._status(
            DIAGNOSTIC_SCAN,
            OK,
            'Scan input is healthy',
            values,
        )

    def _tf_status(self):
        values = {
            'map_frame': self.map_frame,
            'odom_frame': self.odom_frame,
            'base_frame': self.base_frame,
            'scan_frame': self.scan_frame,
        }
        try:
            map_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
            self.tf_buffer.lookup_transform(
                self.base_frame,
                self.scan_frame,
                Time(),
            )
        except TransformException as ex:
            values['error'] = str(ex)
            return self._status(
                DIAGNOSTIC_TF,
                ERROR,
                'SLAM TF chain is unavailable',
                values,
            )

        stamp_ns = Time.from_msg(map_base.header.stamp).nanoseconds
        now_ns = self.get_clock().now().nanoseconds
        age = max(0.0, float(now_ns - stamp_ns) / 1e9)
        values['map_to_base_age_sec'] = '%.3f' % age
        if age >= self.tf_error_age_sec:
            return self._status(
                DIAGNOSTIC_TF,
                ERROR,
                'SLAM TF chain is stale (%.2f s)' % age,
                values,
            )
        if age >= self.tf_warn_age_sec:
            return self._status(
                DIAGNOSTIC_TF,
                WARN,
                'SLAM TF chain is delayed (%.2f s)' % age,
                values,
            )
        return self._status(
            DIAGNOSTIC_TF,
            OK,
            'SLAM TF chain is healthy',
            values,
        )

    def _map_status(self):
        values = {
            'topic': self.map_topic,
            'width': str(self.map_width),
            'height': str(self.map_height),
        }
        if not self.map_received:
            return self._status(
                DIAGNOSTIC_MAP,
                ERROR,
                'No map received on %s' % self.map_topic,
                values,
            )
        if self.map_width <= 0 or self.map_height <= 0:
            return self._status(
                DIAGNOSTIC_MAP,
                ERROR,
                'Received map is empty',
                values,
            )
        return self._status(
            DIAGNOSTIC_MAP,
            OK,
            'Map is available',
            values,
        )

    def _jump_status(self):
        if not self.jump_events:
            return self._status(
                DIAGNOSTIC_JUMP,
                OK,
                'No recent map to odom jumps detected',
                {'window_sec': '%.1f' % self.jump_hold_sec},
            )

        event = max(
            self.jump_events,
            key=lambda item: (item[1], item[2], item[3]),
        )
        level = event[1]
        translation_step = event[2]
        yaw_step = event[3]
        values = {
            'event_count': str(len(self.jump_events)),
            'max_translation_step_m': '%.3f' % translation_step,
            'max_yaw_step_deg': '%.2f' % yaw_step,
            'window_sec': '%.1f' % self.jump_hold_sec,
        }
        return self._status(
            DIAGNOSTIC_JUMP,
            level,
            'Localization correction jump: %.3f m, %.2f deg'
            % (translation_step, yaw_step),
            values,
        )

    def _timing_status(self):
        total = len(self.scan_tf_results)
        failures = sum(
            1 for _, available in self.scan_tf_results if not available
        )
        ratio = float(failures) / float(total) if total else 0.0
        values = {
            'checked_scans': str(total),
            'tf_failures': str(failures),
            'failure_ratio': '%.3f' % ratio,
            'window_sec': '%.1f' % self.timing_window_sec,
            'check_delay_sec': '%.2f' % self.scan_tf_check_delay_sec,
        }
        if total < 3:
            return self._status(
                DIAGNOSTIC_TIMING,
                WARN,
                'Not enough scans for timing evaluation',
                values,
            )
        if ratio >= self.timing_error_failure_ratio:
            return self._status(
                DIAGNOSTIC_TIMING,
                ERROR,
                'TF was unavailable at scan time for %.1f%% of scans'
                % (ratio * 100.0),
                values,
            )
        if ratio >= self.timing_warn_failure_ratio:
            return self._status(
                DIAGNOSTIC_TIMING,
                WARN,
                'Intermittent scan/TF timing failures (%.1f%%)'
                % (ratio * 100.0),
                values,
            )
        return self._status(
            DIAGNOSTIC_TIMING,
            OK,
            'Scan and TF timing is healthy',
            values,
        )

    def _node_names(self):
        result = set()
        for name, namespace in self.get_node_names_and_namespaces():
            if namespace == '/':
                result.add('/' + name)
            else:
                result.add(namespace.rstrip('/') + '/' + name)
        return result

    @staticmethod
    def _status(name, level, message, values):
        status = DiagnosticStatus()
        status.level = bytes([int(level)])
        status.name = name
        status.message = message
        status.hardware_id = 'agar_slam'
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in sorted(values.items())
        ]
        return status


def main(args=None):
    """Run the SLAM health monitor node."""
    rclpy.init(args=args)
    node = SlamHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
