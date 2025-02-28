import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import time

from geometry_msgs.msg import TransformStamped

class RosbagTransformations(Node):
    def __init__(self):
        super().__init__('rosbag_transformations')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.run()


    def run(self):
        while rclpy.ok():
            self.get_logger().info('Publishing transforms')
            
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))

            time.sleep(1/20)

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
    rosbag_transform = RosbagTransformations()
    rclpy.spin(rosbag_transform)
    rosbag_transform.destroy_node()
    rclpy.shutdown()

    rosbag_transform.get_logger().info('Closed')

if __name__ == '__main__':
    main()