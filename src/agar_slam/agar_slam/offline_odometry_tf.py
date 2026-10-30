#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


class OfflineOdometryTf(Node):

    def __init__(self):
        super().__init__('offline_odometry_tf')

        self.declare_parameter('input_topic', '/odometry/filtered')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')

        input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.message_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            input_topic,
            self.odometry_callback,
            qos,
        )
        self.get_logger().info(
            'Replaying %s as TF %s -> %s'
            % (input_topic, self.parent_frame, self.child_frame)
        )

    def odometry_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info('Published first offline odometry transform.')


def main(args=None):
    rclpy.init(args=args)
    node = OfflineOdometryTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(
            'Offline odometry TF finished: processed={}'.format(node.message_count),
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
