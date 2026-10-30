#!/usr/bin/env python3

import csv
from datetime import datetime, timezone
import json
import math
import os
import re

from geometry_msgs.msg import PoseWithCovarianceStamped
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
from sensor_msgs.msg import LaserScan, NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def planar_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (translation.x, translation.y, yaw_from_quaternion(rotation))


def pose_step(previous, current):
    if previous is None or current is None:
        return (0.0, 0.0)

    distance = math.hypot(
        current[0] - previous[0],
        current[1] - previous[1],
    )
    yaw = abs(normalize_angle(current[2] - previous[2]))
    return (distance, yaw)


def safe_filename(value):
    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(value).strip())
    return value.strip('._') or 'offline_run'


class OfflineLocalizationEvaluator(Node):

    def __init__(self):
        super().__init__('offline_localization_evaluator')

        self.declare_parameter('localizer', 'baseline')
        self.declare_parameter('run_name', 'offline_run')
        self.declare_parameter(
            'output_dir',
            '/home/nvidia/workspace/slam_results',
        )
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('translation_jump_m', 0.25)
        self.declare_parameter('yaw_jump_deg', 5.0)

        self.localizer = self.get_parameter('localizer').value
        self.run_name = safe_filename(self.get_parameter('run_name').value)
        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        self.sample_rate_hz = float(
            self.get_parameter('sample_rate_hz').value
        )
        self.translation_jump_m = float(
            self.get_parameter('translation_jump_m').value
        )
        self.yaw_jump_rad = math.radians(float(
            self.get_parameter('yaw_jump_deg').value
        ))

        if self.sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be greater than zero')

        os.makedirs(self.output_dir, exist_ok=True)
        wall_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        result_base = '{}_{}_{}'.format(
            self.run_name,
            safe_filename(self.localizer),
            wall_stamp,
        )
        self.csv_path = os.path.join(
            self.output_dir,
            result_base + '_trajectory.csv',
        )
        self.summary_path = os.path.join(
            self.output_dir,
            result_base + '_summary.json',
        )

        self.csv_file = open(
            self.csv_path,
            'w',
            newline='',
            encoding='utf-8',
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'ros_time_sec',
            'map_odom_x',
            'map_odom_y',
            'map_odom_yaw_deg',
            'map_odom_step_m',
            'map_odom_step_deg',
            'map_odom_jump',
            'odom_base_x',
            'odom_base_y',
            'odom_base_yaw_deg',
            'map_base_x',
            'map_base_y',
            'map_base_yaw_deg',
            'amcl_cov_x',
            'amcl_cov_y',
            'amcl_cov_yaw',
            'gps_status',
            'gps_latitude',
            'gps_longitude',
        ])

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
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
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            sensor_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            latched_qos,
        )

        self.scan_count = 0
        self.zed_odom_count = 0
        self.filtered_odom_count = 0
        self.gps_count = 0
        self.amcl_pose_count = 0

        self.latest_gps = None
        self.latest_amcl_covariance = None
        self.max_amcl_covariance = [0.0, 0.0, 0.0]

        self.sample_count = 0
        self.valid_map_odom_samples = 0
        self.valid_odom_base_samples = 0
        self.valid_map_base_samples = 0
        self.time_reset_count = 0
        self.jump_events = []
        self.previous_map_odom = None
        self.first_map_base = None
        self.last_map_base = None
        self.first_ros_time = None
        self.last_ros_time = None
        self.max_translation_step = 0.0
        self.max_yaw_step = 0.0
        self.cumulative_translation_correction = 0.0
        self.cumulative_yaw_correction = 0.0
        self.finalized = False

        self.create_timer(1.0 / self.sample_rate_hz, self.sample)
        self.get_logger().info(
            'Offline localization evaluator started: localizer=%s csv=%s'
            % (self.localizer, self.csv_path)
        )

    def scan_callback(self, _msg):
        self.scan_count += 1

    def zed_odom_callback(self, _msg):
        self.zed_odom_count += 1

    def filtered_odom_callback(self, _msg):
        self.filtered_odom_count += 1

    def gps_callback(self, msg):
        self.gps_count += 1
        self.latest_gps = (
            int(msg.status.status),
            float(msg.latitude),
            float(msg.longitude),
        )

    def amcl_pose_callback(self, msg):
        self.amcl_pose_count += 1
        covariance = msg.pose.covariance
        self.latest_amcl_covariance = (
            float(covariance[0]),
            float(covariance[7]),
            float(covariance[35]),
        )
        for index, value in enumerate(self.latest_amcl_covariance):
            self.max_amcl_covariance[index] = max(
                self.max_amcl_covariance[index],
                abs(value),
            )

    def lookup_pose(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
            return planar_pose(transform)
        except TransformException:
            return None

    def sample(self):
        ros_time = self.get_clock().now().nanoseconds / 1e9
        if ros_time <= 0.0:
            return

        if self.last_ros_time is not None and ros_time < self.last_ros_time:
            self.time_reset_count += 1
            self.previous_map_odom = None

        if self.first_ros_time is None:
            self.first_ros_time = ros_time
        self.last_ros_time = ros_time

        map_odom = self.lookup_pose('map', 'odom')
        odom_base = self.lookup_pose('odom', 'base_footprint')
        map_base = self.lookup_pose('map', 'base_footprint')

        if map_odom is not None:
            self.valid_map_odom_samples += 1
        if odom_base is not None:
            self.valid_odom_base_samples += 1
        if map_base is not None:
            self.valid_map_base_samples += 1
            if self.first_map_base is None:
                self.first_map_base = map_base
            self.last_map_base = map_base

        step_m, step_yaw = pose_step(self.previous_map_odom, map_odom)
        is_jump = (
            self.previous_map_odom is not None
            and map_odom is not None
            and (
                step_m >= self.translation_jump_m
                or step_yaw >= self.yaw_jump_rad
            )
        )

        if self.previous_map_odom is not None and map_odom is not None:
            self.max_translation_step = max(
                self.max_translation_step,
                step_m,
            )
            self.max_yaw_step = max(self.max_yaw_step, step_yaw)
            self.cumulative_translation_correction += step_m
            self.cumulative_yaw_correction += step_yaw

        if is_jump:
            self.jump_events.append({
                'ros_time_sec': ros_time,
                'translation_m': step_m,
                'yaw_deg': math.degrees(step_yaw),
                'map_odom_x': map_odom[0],
                'map_odom_y': map_odom[1],
                'map_odom_yaw_deg': math.degrees(map_odom[2]),
            })

        if map_odom is not None:
            self.previous_map_odom = map_odom

        covariance = self.latest_amcl_covariance or ('', '', '')
        gps = self.latest_gps or ('', '', '')
        self.csv_writer.writerow([
            ros_time,
            self.pose_value(map_odom, 0),
            self.pose_value(map_odom, 1),
            self.pose_degrees(map_odom),
            step_m if map_odom is not None else '',
            math.degrees(step_yaw) if map_odom is not None else '',
            int(is_jump),
            self.pose_value(odom_base, 0),
            self.pose_value(odom_base, 1),
            self.pose_degrees(odom_base),
            self.pose_value(map_base, 0),
            self.pose_value(map_base, 1),
            self.pose_degrees(map_base),
            covariance[0],
            covariance[1],
            covariance[2],
            gps[0],
            gps[1],
            gps[2],
        ])
        self.sample_count += 1
        if self.sample_count % 10 == 0:
            self.csv_file.flush()

    @staticmethod
    def pose_value(pose, index):
        return '' if pose is None else pose[index]

    @staticmethod
    def pose_degrees(pose):
        return '' if pose is None else math.degrees(pose[2])

    @staticmethod
    def serializable_pose(pose):
        if pose is None:
            return None
        return {
            'x': pose[0],
            'y': pose[1],
            'yaw_deg': math.degrees(pose[2]),
        }

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True

        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

        endpoint_distance = None
        endpoint_yaw = None
        if self.first_map_base is not None and self.last_map_base is not None:
            endpoint_distance, endpoint_yaw = pose_step(
                self.first_map_base,
                self.last_map_base,
            )

        duration = None
        if self.first_ros_time is not None and self.last_ros_time is not None:
            duration = max(0.0, self.last_ros_time - self.first_ros_time)

        summary = {
            'localizer': self.localizer,
            'run_name': self.run_name,
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'ros_time': {
                'start_sec': self.first_ros_time,
                'end_sec': self.last_ros_time,
                'duration_sec': duration,
                'reset_count': self.time_reset_count,
            },
            'message_counts': {
                'scan': self.scan_count,
                'zed_odom': self.zed_odom_count,
                'filtered_odom': self.filtered_odom_count,
                'gps_fix': self.gps_count,
                'amcl_pose': self.amcl_pose_count,
            },
            'transform_samples': {
                'total': self.sample_count,
                'map_to_odom': self.valid_map_odom_samples,
                'odom_to_base_footprint': self.valid_odom_base_samples,
                'map_to_base_footprint': self.valid_map_base_samples,
            },
            'map_to_odom_corrections': {
                'translation_jump_threshold_m': self.translation_jump_m,
                'yaw_jump_threshold_deg': math.degrees(self.yaw_jump_rad),
                'jump_count': len(self.jump_events),
                'max_translation_step_m': self.max_translation_step,
                'max_yaw_step_deg': math.degrees(self.max_yaw_step),
                'cumulative_translation_m': (
                    self.cumulative_translation_correction
                ),
                'cumulative_yaw_deg': math.degrees(
                    self.cumulative_yaw_correction
                ),
                'events': self.jump_events,
            },
            'map_to_base_footprint': {
                'first': self.serializable_pose(self.first_map_base),
                'last': self.serializable_pose(self.last_map_base),
                'endpoint_displacement_m': endpoint_distance,
                'endpoint_yaw_difference_deg': (
                    None
                    if endpoint_yaw is None
                    else math.degrees(endpoint_yaw)
                ),
            },
            'amcl_covariance': {
                'last': self.latest_amcl_covariance,
                'maximum_absolute': self.max_amcl_covariance,
            },
            'files': {
                'trajectory_csv': self.csv_path,
                'summary_json': self.summary_path,
            },
        }

        with open(self.summary_path, 'w', encoding='utf-8') as summary_file:
            json.dump(summary, summary_file, indent=2, sort_keys=True)
            summary_file.write('\n')

        message = (
            'Offline localization result: jumps=%d max_step=%.3fm/%.2fdeg '
            'summary=%s'
            % (
                len(self.jump_events),
                self.max_translation_step,
                math.degrees(self.max_yaw_step),
                self.summary_path,
            )
        )
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(message)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineLocalizationEvaluator()
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
