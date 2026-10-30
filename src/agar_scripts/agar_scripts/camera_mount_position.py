import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


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
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


class CameraMountPosition(Node):

    def __init__(self):
        super().__init__('camera_position')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('mount_frame', 'main_camera_link')
        self.declare_parameter('front_camera_frame', 'zedx_camera_link')
        self.declare_parameter('rear_camera_frame', 'zedxm_camera_link')
        self.declare_parameter('mount_x_m', 0.55)
        self.declare_parameter('mount_y_m', 0.0)
        self.declare_parameter('mount_z_m', 1.60)
        self.declare_parameter('mount_roll_rad', 0.0)
        self.declare_parameter('mount_pitch_rad', 0.0)
        self.declare_parameter('mount_yaw_rad', 0.0)
        self.declare_parameter('front_x_m', 0.05)
        self.declare_parameter('rear_x_m', -0.05)

        self.base_frame = self.get_parameter('base_frame').value
        self.mount_frame = self.get_parameter('mount_frame').value
        self.front_camera_frame = self.get_parameter(
            'front_camera_frame'
        ).value
        self.rear_camera_frame = self.get_parameter(
            'rear_camera_frame'
        ).value
        self.mount_position = [
            self.get_parameter('mount_x_m').value,
            self.get_parameter('mount_y_m').value,
            self.get_parameter('mount_z_m').value,
        ]
        self.mount_orientation = quaternion_from_euler(
            self.get_parameter('mount_roll_rad').value,
            self.get_parameter('mount_pitch_rad').value,
            self.get_parameter('mount_yaw_rad').value,
        )
        self.front_x_m = self.get_parameter('front_x_m').value
        self.rear_x_m = self.get_parameter('rear_x_m').value

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_tf_data()

        self.get_logger().info(
            'Published fixed camera mount TF: %s -> %s -> %s'
            % (
                self.base_frame,
                self.mount_frame,
                self.front_camera_frame,
            )
        )

    def publish_tf_data(self):
        transforms = [
            self.set_transform(
                self.base_frame,
                self.mount_frame,
                self.mount_position,
                self.mount_orientation,
            ),
            self.set_transform(
                self.mount_frame,
                self.front_camera_frame,
                [self.front_x_m, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ),
            self.set_transform(
                self.mount_frame,
                self.rear_camera_frame,
                [self.rear_x_m, 0.0, 0.0],
                quaternion_from_euler(0.0, 0.0, 3.141592653589793),
            ),
        ]
        self.tf_broadcaster.sendTransform(transforms)

    def set_transform(self, frame_id, child_frame_id, position, orientation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        return t


def main(args=None):
    rclpy.init(args=args)
    camera_position = CameraMountPosition()
    try:
        rclpy.spin(camera_position)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        camera_position.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
