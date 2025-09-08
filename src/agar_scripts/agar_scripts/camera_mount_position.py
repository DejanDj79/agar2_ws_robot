import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Imu
from math import pi

class CameraMountPosition(Node):

    def __init__(self):
        super().__init__('camera_position')

        timer_period = 1.0 / 20
        self.timer_camera_tf = self.create_timer(timer_period, self.publish_tf_data)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_orientation = [0.0, 0.0, 0.0, 1.0]


        self.zedxm_orientation = quaternion_from_euler(0, 0, pi)

        self.get_logger().info('Initialization completed')

        self.imu_sub  = self.create_subscription(Imu, '/zedx/zed_node/imu/data', self.imu_data_callback, 10) 

    def imu_data_callback(self, msg):
        self.camera_orientation[0] = msg.orientation.x
        self.camera_orientation[1] = msg.orientation.y
        self.camera_orientation[2] = msg.orientation.z
        self.camera_orientation[3] = msg.orientation.w

        self.get_logger().info(f'IMU data received: {self.camera_orientation[0]}, {self.camera_orientation[1]}, {self.camera_orientation[2]}, {self.camera_orientation[3]}')

    def publish_tf_data(self):
        self.tf_broadcaster.sendTransform(self.set_transform('top_stand', 'main_camera_link', [0.55, 0.0, 0.25], self.camera_orientation))
        self.tf_broadcaster.sendTransform(self.set_transform('main_camera_link', 'zedx_camera_link', [0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        self.tf_broadcaster.sendTransform(self.set_transform('main_camera_link', 'zedxm_camera_link', [-0.05, 0.0, 0.0], self.zedxm_orientation))


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
    rclpy.spin(camera_position)
    camera_position.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()