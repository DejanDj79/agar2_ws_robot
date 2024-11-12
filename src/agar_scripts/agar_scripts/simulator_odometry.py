import rclpy
from tf2_ros import TransformBroadcaster
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion 

from agar_common.agar_common_library.alvinxy import ll2xy

class SimulatorOdometry(Node):

    def __init__(self):
        super().__init__('simulator_odometry')

        self.get_logger().info('Initialization completed')

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.publish_transform)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.gps_lat_origin = 43.330890405343304
        self.gps_lon_origin = 21.893977363705066

        self.position = [0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        self.odom_pub = self.create_publisher(Odometry, 'odometry/agar_odom', 10)

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_subscriber_callback,
            10)
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_subscriber_callback,
            10)
        
    def gps_subscriber_callback(self, msg):
        lat = msg.latitude
        lng = msg.longitude
        self.position = ll2xy(lat, lng, self.gps_lat_origin, self.gps_lon_origin)
        # self.get_logger().info('Latitude: {0}, longitude: {1}, pos x: {2}, pos y: {3}'.format(lat, lng, self.position[0], self.position[1]))
        # self.get_logger().info('pos x: {0}, pos y: {1}'.format(self.position[0], self.position[1]))

    def imu_subscriber_callback(self, msg):
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # self.get_logger().info('x: {0}, y: {1}, z: {2}, w: {3}'.format(self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]))

    def publish_transform(self):
        # self.get_logger().info('pos x: {0}, pos y: {1}'.format(self.position[0], self.position[1]))
        # self.get_logger().info('x: {0}, y: {1}, z: {2}, w: {3}'.format(self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]))
        self.tf_broadcaster.sendTransform(self.set_transform('odom', 'base_footprint', self.position, self.orientation))
        self.tf_broadcaster.sendTransform(self.set_transform('map', 'odom', [0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.orientation.x = self.orientation[0]
        odom.pose.pose.orientation.y = self.orientation[1]
        odom.pose.pose.orientation.z = self.orientation[2]
        odom.pose.pose.orientation.w = self.orientation[3]
        self.odom_pub.publish(odom)

    def set_transform(self, frame_id, child_frame_id, position, orientation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        return t



def main(args=None):
    rclpy.init(args=args)
    simulator_odometry = SimulatorOdometry()
    # simulator_odometry.start()
    rclpy.spin(simulator_odometry)
    simulator_odometry.destroy_node()
    rclpy.shutdown()

    simulator_odometry.get_logger().info('Closed')

if __name__ == '__main__':
    main()