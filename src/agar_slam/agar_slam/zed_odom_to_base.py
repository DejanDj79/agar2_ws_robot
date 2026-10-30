#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
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
from tf2_ros import TransformBroadcaster


def normalized_quaternion(quaternion):
    values = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(values)
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero')
    return values / norm


def quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_inverse(quaternion):
    quaternion = normalized_quaternion(quaternion)
    return np.array([
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    ])


def rotation_matrix(quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def quaternion_angular_distance(first, second):
    first = normalized_quaternion(first)
    second = normalized_quaternion(second)
    dot = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    return normalized_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def skew(vector):
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def camera_pose_to_base(camera_position, camera_orientation,
                        base_to_camera_position, base_to_camera_orientation):
    camera_orientation = normalized_quaternion(camera_orientation)
    base_to_camera_orientation = normalized_quaternion(
        base_to_camera_orientation
    )
    base_orientation = normalized_quaternion(quaternion_multiply(
        camera_orientation,
        quaternion_inverse(base_to_camera_orientation),
    ))
    offset_in_world = (
        rotation_matrix(base_orientation)
        @ np.asarray(base_to_camera_position, dtype=float)
    )
    base_position = np.asarray(camera_position, dtype=float) - offset_in_world
    return base_position, base_orientation


def camera_twist_to_base(linear_velocity, angular_velocity,
                         base_to_camera_position,
                         base_to_camera_orientation):
    rotation = rotation_matrix(base_to_camera_orientation)
    angular_base = rotation @ np.asarray(angular_velocity, dtype=float)
    linear_camera_in_base = rotation @ np.asarray(
        linear_velocity,
        dtype=float,
    )
    offset = np.asarray(base_to_camera_position, dtype=float)
    linear_base = linear_camera_in_base + skew(offset) @ angular_base
    return linear_base, angular_base


def transform_twist_covariance(covariance, base_to_camera_position,
                               base_to_camera_orientation):
    covariance = np.asarray(covariance, dtype=float).reshape((6, 6))
    rotation = rotation_matrix(base_to_camera_orientation)
    offset = np.asarray(base_to_camera_position, dtype=float)
    transform = np.zeros((6, 6), dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3:] = skew(offset) @ rotation
    transform[3:, 3:] = rotation
    return transform @ covariance @ transform.T


def rotate_pose_covariance(covariance, orientation):
    covariance = np.asarray(covariance, dtype=float).reshape((6, 6))
    rotation = rotation_matrix(orientation)
    transform = np.zeros((6, 6), dtype=float)
    transform[:3, :3] = rotation
    transform[3:, 3:] = rotation
    return transform @ covariance @ transform.T


class PoseContinuityGuard:

    def __init__(self, enabled=True, translation_threshold_m=0.5,
                 yaw_threshold_deg=10.0):
        if translation_threshold_m <= 0.0:
            raise ValueError('translation threshold must be greater than zero')
        if yaw_threshold_deg <= 0.0:
            raise ValueError('yaw threshold must be greater than zero')

        self.enabled = bool(enabled)
        self.translation_threshold_m = float(translation_threshold_m)
        self.rotation_threshold_rad = math.radians(float(yaw_threshold_deg))
        self.correction_position = np.zeros(3, dtype=float)
        self.correction_orientation = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=float,
        )
        self.previous_raw_position = None
        self.previous_raw_orientation = None
        self.previous_output_position = None
        self.previous_output_orientation = None
        self.reset_count = 0

    def correct(self, raw_position, raw_orientation):
        raw_position = np.asarray(raw_position, dtype=float)
        raw_orientation = normalized_quaternion(raw_orientation)
        translation_step = 0.0
        rotation_step = 0.0
        discontinuity = False

        if self.previous_raw_position is not None:
            translation_step = float(np.linalg.norm(
                raw_position - self.previous_raw_position
            ))
            rotation_step = quaternion_angular_distance(
                raw_orientation,
                self.previous_raw_orientation,
            )
            discontinuity = self.enabled and (
                translation_step > self.translation_threshold_m
                or rotation_step > self.rotation_threshold_rad
            )

        if discontinuity:
            self.correction_orientation = normalized_quaternion(
                quaternion_multiply(
                    self.previous_output_orientation,
                    quaternion_inverse(raw_orientation),
                )
            )
            self.correction_position = (
                self.previous_output_position
                - rotation_matrix(self.correction_orientation) @ raw_position
            )
            self.reset_count += 1

        output_orientation = normalized_quaternion(quaternion_multiply(
            self.correction_orientation,
            raw_orientation,
        ))
        output_position = (
            self.correction_position
            + rotation_matrix(self.correction_orientation) @ raw_position
        )

        self.previous_raw_position = raw_position.copy()
        self.previous_raw_orientation = raw_orientation.copy()
        self.previous_output_position = output_position.copy()
        self.previous_output_orientation = output_orientation.copy()
        return (
            output_position,
            output_orientation,
            discontinuity,
            translation_step,
            math.degrees(rotation_step),
        )


class ZedOdomToBase(Node):

    def __init__(self):
        super().__init__('zed_odom_to_base')

        self.declare_parameter('input_topic', '/zedx/zed_node/odom')
        self.declare_parameter('output_topic', '/zedx/odom_base')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('sensor_frame', 'zedx_camera_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('tf_time_offset_sec', 0.0)
        self.declare_parameter('preserve_odom_continuity', True)
        self.declare_parameter('reset_translation_threshold_m', 0.5)
        self.declare_parameter('reset_yaw_threshold_deg', 10.0)
        self.declare_parameter('sensor_x_m', 0.6)
        self.declare_parameter('sensor_y_m', 0.0)
        self.declare_parameter('sensor_z_m', 1.6)
        self.declare_parameter('sensor_roll_rad', 0.0)
        self.declare_parameter('sensor_pitch_rad', 0.0)
        self.declare_parameter('sensor_yaw_rad', 0.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_time_offset_sec = float(
            self.get_parameter('tf_time_offset_sec').value
        )
        self.preserve_odom_continuity = bool(
            self.get_parameter('preserve_odom_continuity').value
        )
        self.reset_translation_threshold_m = float(
            self.get_parameter('reset_translation_threshold_m').value
        )
        self.reset_yaw_threshold_deg = float(
            self.get_parameter('reset_yaw_threshold_deg').value
        )
        self.base_to_camera_position = np.array([
            float(self.get_parameter('sensor_x_m').value),
            float(self.get_parameter('sensor_y_m').value),
            float(self.get_parameter('sensor_z_m').value),
        ])
        self.base_to_camera_orientation = quaternion_from_euler(
            float(self.get_parameter('sensor_roll_rad').value),
            float(self.get_parameter('sensor_pitch_rad').value),
            float(self.get_parameter('sensor_yaw_rad').value),
        )
        self.continuity_guard = PoseContinuityGuard(
            enabled=self.preserve_odom_continuity,
            translation_threshold_m=self.reset_translation_threshold_m,
            yaw_threshold_deg=self.reset_yaw_threshold_deg,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Odometry,
            self.output_topic,
            qos,
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.create_subscription(
            Odometry,
            self.input_topic,
            self.odometry_callback,
            qos,
        )
        self.input_count = 0
        self.output_count = 0
        self.rejected_count = 0

        self.get_logger().info(
            'Converting %s (%s) to %s (%s), sensor xyz=[%.3f, %.3f, %.3f], '
            'publish_tf=%s offset=%.3fs preserve_continuity=%s '
            'reset_thresholds=[%.3fm, %.1fdeg]'
            % (
                self.input_topic,
                self.sensor_frame,
                self.output_topic,
                self.base_frame,
                *self.base_to_camera_position,
                self.publish_tf,
                self.tf_time_offset_sec,
                self.preserve_odom_continuity,
                self.reset_translation_threshold_m,
                self.reset_yaw_threshold_deg,
            )
        )

    @staticmethod
    def message_quaternion(quaternion):
        return np.array([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ])

    @staticmethod
    def message_vector(vector):
        return np.array([vector.x, vector.y, vector.z])

    def publish_base_transform(self, odometry):
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.frame_id = odometry.header.frame_id
        transform.header.stamp = (
            Time.from_msg(odometry.header.stamp) + Duration(
                seconds=self.tf_time_offset_sec
            )
        ).to_msg()
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def odometry_callback(self, msg):
        self.input_count += 1
        if msg.header.frame_id != self.odom_frame:
            self.rejected_count += 1
            if self.rejected_count == 1:
                self.get_logger().error(
                    'Expected frame_id %s, received %s'
                    % (self.odom_frame, msg.header.frame_id)
                )
            return
        if msg.child_frame_id != self.sensor_frame:
            self.rejected_count += 1
            if self.rejected_count == 1:
                self.get_logger().error(
                    'Expected child_frame_id %s, received %s'
                    % (self.sensor_frame, msg.child_frame_id)
                )
            return

        try:
            base_position, base_orientation = camera_pose_to_base(
                self.message_vector(msg.pose.pose.position),
                self.message_quaternion(msg.pose.pose.orientation),
                self.base_to_camera_position,
                self.base_to_camera_orientation,
            )
            (
                base_position,
                base_orientation,
                discontinuity,
                translation_step,
                rotation_step_deg,
            ) = self.continuity_guard.correct(
                base_position,
                base_orientation,
            )
            linear_base, angular_base = camera_twist_to_base(
                self.message_vector(msg.twist.twist.linear),
                self.message_vector(msg.twist.twist.angular),
                self.base_to_camera_position,
                self.base_to_camera_orientation,
            )
            twist_covariance = transform_twist_covariance(
                msg.twist.covariance,
                self.base_to_camera_position,
                self.base_to_camera_orientation,
            )
            pose_covariance = rotate_pose_covariance(
                msg.pose.covariance,
                self.continuity_guard.correction_orientation,
            )
        except ValueError as error:
            self.rejected_count += 1
            if self.rejected_count == 1:
                self.get_logger().error('Invalid ZED odometry: %s' % error)
            return

        if discontinuity:
            self.get_logger().warning(
                'Detected ZED odometry discontinuity #%d: %.3f m, %.2f deg; '
                'preserving continuous odom output.'
                % (
                    self.continuity_guard.reset_count,
                    translation_step,
                    rotation_step_deg,
                )
            )

        output = Odometry()
        output.header = msg.header
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = float(base_position[0])
        output.pose.pose.position.y = float(base_position[1])
        output.pose.pose.position.z = float(base_position[2])
        output.pose.pose.orientation.x = float(base_orientation[0])
        output.pose.pose.orientation.y = float(base_orientation[1])
        output.pose.pose.orientation.z = float(base_orientation[2])
        output.pose.pose.orientation.w = float(base_orientation[3])

        # The camera-to-base lever-arm covariance is not modeled yet. Rotate
        # the supplied covariance when a parent-frame continuity correction
        # is active; twist covariance is transformed exactly.
        output.pose.covariance = pose_covariance.reshape(36).tolist()
        output.twist.twist.linear.x = float(linear_base[0])
        output.twist.twist.linear.y = float(linear_base[1])
        output.twist.twist.linear.z = float(linear_base[2])
        output.twist.twist.angular.x = float(angular_base[0])
        output.twist.twist.angular.y = float(angular_base[1])
        output.twist.twist.angular.z = float(angular_base[2])
        output.twist.covariance = twist_covariance.reshape(36).tolist()

        self.publisher.publish(output)
        self.publish_base_transform(output)
        self.output_count += 1
        if self.output_count == 1:
            self.get_logger().info('Published first base-frame ZED odometry.')


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdomToBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(
            'ZED odometry conversion finished: input={} output={} rejected={} '
            'continuity_resets={}'
            .format(
                node.input_count,
                node.output_count,
                node.rejected_count,
                node.continuity_guard.reset_count,
            ),
            flush=True,
        )
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
