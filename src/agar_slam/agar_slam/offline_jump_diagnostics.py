#!/usr/bin/env python3

from datetime import datetime, timezone
import json
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from agar_slam.offline_localization_evaluator import (
    normalize_angle,
    planar_pose,
    pose_step,
    safe_filename,
    yaw_from_quaternion,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def quaternion_is_valid(quaternion):
    norm_squared = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    return norm_squared > 0.5


def angle_delta_degrees(first, second):
    if first is None or second is None:
        return None
    return math.degrees(normalize_angle(math.radians(first - second)))


class OfflineJumpDiagnostics(Node):

    def __init__(self):
        super().__init__('offline_jump_diagnostics')

        self.declare_parameter('localizer', 'baseline')
        self.declare_parameter('run_name', 'offline_run')
        self.declare_parameter(
            'output_dir',
            '/home/nvidia/workspace/slam_results',
        )
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('translation_jump_m', 0.25)
        self.declare_parameter('yaw_jump_deg', 5.0)

        self.localizer = str(self.get_parameter('localizer').value)
        self.run_name = safe_filename(self.get_parameter('run_name').value)
        self.output_dir = os.path.expanduser(
            str(self.get_parameter('output_dir').value)
        )
        self.sample_rate_hz = float(
            self.get_parameter('sample_rate_hz').value
        )
        self.translation_jump_m = float(
            self.get_parameter('translation_jump_m').value
        )
        self.yaw_jump_rad = math.radians(
            float(self.get_parameter('yaw_jump_deg').value)
        )

        if self.sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be greater than zero')

        os.makedirs(self.output_dir, exist_ok=True)
        wall_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        result_base = '{}_{}_{}_jump_diagnostics.json'.format(
            self.run_name,
            safe_filename(self.localizer),
            wall_stamp,
        )
        self.output_path = os.path.join(self.output_dir, result_base)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            '/zedx/zed_node/odom',
            self.zed_odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            '/zedx/zed_node/imu/data',
            self.zed_imu_callback,
            sensor_qos,
        )

        self.message_counts = {
            'scan': 0,
            'cmd_vel': 0,
            'zed_odom': 0,
            'filtered_odom': 0,
            'imu': 0,
            'zed_imu': 0,
        }
        self.latest_scan = None
        self.latest_cmd_vel = None
        self.latest_zed_odom = None
        self.latest_filtered_odom = None
        self.latest_imu = None
        self.latest_zed_imu = None

        self.events = []
        self.previous_map_odom = None
        self.sample_count = 0
        self.valid_map_odom_samples = 0
        self.time_reset_count = 0
        self.first_ros_time = None
        self.last_ros_time = None
        self.finalized = False

        self.create_timer(1.0 / self.sample_rate_hz, self.sample)
        self.get_logger().info(
            'Offline jump diagnostics started: localizer=%s output=%s'
            % (self.localizer, self.output_path)
        )

    def current_ros_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg):
        self.message_counts['scan'] += 1
        self.latest_scan = {
            'stamp_sec': stamp_seconds(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'range_count': len(msg.ranges),
            'finite_range_count': sum(
                1 for value in msg.ranges if math.isfinite(value)
            ),
        }

    def cmd_vel_callback(self, msg):
        self.message_counts['cmd_vel'] += 1
        self.latest_cmd_vel = {
            'stamp_sec': self.current_ros_time(),
            'linear_x_mps': float(msg.linear.x),
            'linear_y_mps': float(msg.linear.y),
            'angular_z_radps': float(msg.angular.z),
        }

    @staticmethod
    def odometry_snapshot(msg):
        orientation = msg.pose.pose.orientation
        return {
            'stamp_sec': stamp_seconds(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'x_m': float(msg.pose.pose.position.x),
            'y_m': float(msg.pose.pose.position.y),
            'yaw_deg': math.degrees(yaw_from_quaternion(orientation)),
            'linear_x_mps': float(msg.twist.twist.linear.x),
            'linear_y_mps': float(msg.twist.twist.linear.y),
            'angular_z_radps': float(msg.twist.twist.angular.z),
            'pose_covariance_x': float(msg.pose.covariance[0]),
            'pose_covariance_y': float(msg.pose.covariance[7]),
            'pose_covariance_yaw': float(msg.pose.covariance[35]),
        }

    def zed_odom_callback(self, msg):
        self.message_counts['zed_odom'] += 1
        self.latest_zed_odom = self.odometry_snapshot(msg)

    def filtered_odom_callback(self, msg):
        self.message_counts['filtered_odom'] += 1
        self.latest_filtered_odom = self.odometry_snapshot(msg)

    @staticmethod
    def imu_snapshot(msg):
        orientation = msg.orientation
        yaw_deg = None
        if quaternion_is_valid(orientation):
            yaw_deg = math.degrees(yaw_from_quaternion(orientation))

        return {
            'stamp_sec': stamp_seconds(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'yaw_deg': yaw_deg,
            'angular_velocity_z_radps': float(msg.angular_velocity.z),
            'linear_acceleration_x_mps2': float(msg.linear_acceleration.x),
            'linear_acceleration_y_mps2': float(msg.linear_acceleration.y),
            'orientation_covariance_yaw': float(msg.orientation_covariance[8]),
            'angular_velocity_covariance_z': float(
                msg.angular_velocity_covariance[8]
            ),
        }

    def imu_callback(self, msg):
        self.message_counts['imu'] += 1
        self.latest_imu = self.imu_snapshot(msg)

    def zed_imu_callback(self, msg):
        self.message_counts['zed_imu'] += 1
        self.latest_zed_imu = self.imu_snapshot(msg)

    def lookup_transform_snapshot(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException:
            return None, None

        pose = planar_pose(transform)
        snapshot = {
            'stamp_sec': stamp_seconds(transform.header.stamp),
            'x_m': pose[0],
            'y_m': pose[1],
            'yaw_deg': math.degrees(pose[2]),
        }
        return pose, snapshot

    @staticmethod
    def snapshot_with_age(snapshot, ros_time):
        if snapshot is None:
            return None

        result = dict(snapshot)
        stamp = result.get('stamp_sec')
        result['age_sec'] = None if stamp is None else ros_time - stamp
        return result

    @staticmethod
    def odometry_comparison(zed_odom, filtered_odom):
        if zed_odom is None or filtered_odom is None:
            return None

        return {
            'position_delta_m': math.hypot(
                filtered_odom['x_m'] - zed_odom['x_m'],
                filtered_odom['y_m'] - zed_odom['y_m'],
            ),
            'yaw_delta_deg': angle_delta_degrees(
                filtered_odom['yaw_deg'],
                zed_odom['yaw_deg'],
            ),
            'angular_velocity_delta_radps': (
                filtered_odom['angular_z_radps']
                - zed_odom['angular_z_radps']
            ),
        }

    @staticmethod
    def imu_comparison(imu, zed_imu):
        if imu is None or zed_imu is None:
            return None

        return {
            'yaw_delta_deg': angle_delta_degrees(
                imu['yaw_deg'],
                zed_imu['yaw_deg'],
            ),
            'angular_velocity_z_delta_radps': (
                imu['angular_velocity_z_radps']
                - zed_imu['angular_velocity_z_radps']
            ),
        }

    def build_event(
        self,
        ros_time,
        step_m,
        step_yaw,
        map_odom_snapshot,
        odom_base_snapshot,
    ):
        zed_odom = self.snapshot_with_age(self.latest_zed_odom, ros_time)
        filtered_odom = self.snapshot_with_age(
            self.latest_filtered_odom,
            ros_time,
        )
        imu = self.snapshot_with_age(self.latest_imu, ros_time)
        zed_imu = self.snapshot_with_age(self.latest_zed_imu, ros_time)

        return {
            'ros_time_sec': ros_time,
            'correction': {
                'translation_m': step_m,
                'yaw_deg': math.degrees(step_yaw),
            },
            'map_to_odom': self.snapshot_with_age(
                map_odom_snapshot,
                ros_time,
            ),
            'odom_to_base_footprint': self.snapshot_with_age(
                odom_base_snapshot,
                ros_time,
            ),
            'scan': self.snapshot_with_age(self.latest_scan, ros_time),
            'cmd_vel': self.snapshot_with_age(
                self.latest_cmd_vel,
                ros_time,
            ),
            'zed_odom': zed_odom,
            'filtered_odom': filtered_odom,
            'imu': imu,
            'zed_imu': zed_imu,
            'odom_comparison': self.odometry_comparison(
                zed_odom,
                filtered_odom,
            ),
            'imu_comparison': self.imu_comparison(imu, zed_imu),
        }

    def sample(self):
        ros_time = self.current_ros_time()
        if ros_time <= 0.0:
            return

        if self.last_ros_time is not None and ros_time < self.last_ros_time:
            self.time_reset_count += 1
            self.previous_map_odom = None

        if self.first_ros_time is None:
            self.first_ros_time = ros_time
        self.last_ros_time = ros_time

        map_odom, map_odom_snapshot = self.lookup_transform_snapshot(
            'map',
            'odom',
        )
        _, odom_base_snapshot = self.lookup_transform_snapshot(
            'odom',
            'base_footprint',
        )
        if map_odom is not None:
            self.valid_map_odom_samples += 1

        step_m, step_yaw = pose_step(self.previous_map_odom, map_odom)
        is_jump = (
            self.previous_map_odom is not None
            and map_odom is not None
            and (
                step_m >= self.translation_jump_m
                or step_yaw >= self.yaw_jump_rad
            )
        )
        if is_jump:
            self.events.append(self.build_event(
                ros_time,
                step_m,
                step_yaw,
                map_odom_snapshot,
                odom_base_snapshot,
            ))

        if map_odom is not None:
            self.previous_map_odom = map_odom
        self.sample_count += 1

    @staticmethod
    def largest_events(events, key):
        return sorted(
            events,
            key=lambda event: event['correction'][key],
            reverse=True,
        )[:5]

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True

        duration = None
        if self.first_ros_time is not None and self.last_ros_time is not None:
            duration = max(0.0, self.last_ros_time - self.first_ros_time)

        report = {
            'schema_version': 1,
            'localizer': self.localizer,
            'run_name': self.run_name,
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'ros_time': {
                'start_sec': self.first_ros_time,
                'end_sec': self.last_ros_time,
                'duration_sec': duration,
                'reset_count': self.time_reset_count,
            },
            'thresholds': {
                'translation_jump_m': self.translation_jump_m,
                'yaw_jump_deg': math.degrees(self.yaw_jump_rad),
            },
            'message_counts': self.message_counts,
            'samples': {
                'total': self.sample_count,
                'valid_map_to_odom': self.valid_map_odom_samples,
            },
            'event_count': len(self.events),
            'largest_translation_events': self.largest_events(
                self.events,
                'translation_m',
            ),
            'largest_yaw_events': self.largest_events(
                self.events,
                'yaw_deg',
            ),
            'events': self.events,
            'files': {
                'jump_diagnostics_json': self.output_path,
            },
        }

        with open(self.output_path, 'w', encoding='utf-8') as output_file:
            json.dump(report, output_file, indent=2, sort_keys=True)
            output_file.write('\n')

        message = (
            'Offline jump diagnostics finished: events=%d output=%s'
            % (len(self.events), self.output_path)
        )
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(message)

    def destroy_node(self):
        self.finalize()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OfflineJumpDiagnostics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
