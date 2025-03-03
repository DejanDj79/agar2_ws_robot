import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import time

from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu

class FakeSensors(Node):
    def __init__(self):
        super().__init__('fake_sensors')

        self.rate = 1/20

        self.tf_broadcaster = TransformBroadcaster(self)

        self.lat    = 43.331053219833336
        self.lng    = 21.890896270916667
        self.alt    = 243.4407

        self.imu_x  = 0.0
        self.imu_y  = 0.0
        self.imu_z  = 0.9999969605322065
        self.imu_w  = 0.0024655478799952496

        self.lon_std_dev = 1.0
        self.lat_std_dev = 1.0
        self.alt_std_dev = 1.0

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        self.run()


    def run(self):
        while rclpy.ok():
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))

            self.tf_broadcaster.sendTransform(self.set_transform('top_stand', 'zed_camera_link', [0.0, 0.0, -0.05], [0.0, 0.0, 0.0, 1.0]))


            gps_fix_msg = NavSatFix()
            gps_fix_msg.header.stamp        = self.get_clock().now().to_msg()
            gps_fix_msg.header.frame_id     = "gps_link"

            gps_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            gps_fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            gps_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            gps_fix_msg.status.service = NavSatStatus.SERVICE_GPS

            gps_fix_msg.latitude    = self.lat
            gps_fix_msg.longitude   = self.lng
            gps_fix_msg.altitude    = self.alt

            hdop = 0.01
            gps_fix_msg.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            gps_fix_msg.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            gps_fix_msg.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.gps_pub.publish(gps_fix_msg)


            imu_msg = Imu()
            imu_msg.header.stamp        = self.get_clock().now().to_msg()
            imu_msg.header.frame_id     = "imu"
            imu_msg.orientation.x       = self.imu_x
            imu_msg.orientation.y       = self.imu_y
            imu_msg.orientation.z       = self.imu_z
            imu_msg.orientation.w       = self.imu_w
            self.imu_pub.publish(imu_msg)



            time.sleep(self.rate)

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
    fake_sensors = FakeSensors()
    rclpy.spin(fake_sensors)
    fake_sensors.destroy_node()
    rclpy.shutdown()

    fake_sensors.get_logger().info('Closed')

if __name__ == '__main__':
    main()