import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_from_euler

import socket
import time
import sys

from math import atan2, pi

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

from agar_common.agar_common_library import gpgga
from agar_common.agar_common_library import alvinxy

from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import TransformStamped

import random

class ToknavGpsDriver(Node):

    def __init__(self):
        super().__init__('toknav_gps')

        self.get_logger().info('Initialization completed')

        timer_period = 0.05  # seconds
        self.timer_gps = self.create_timer(timer_period, self.read_gps)
        self.timer_gps.cancel()

        self.transf_br = TransformBroadcaster(self)

        host = "192.168.0.66"
        port = 66
        self.address = (host, port)
        self.socket = None

        self.connected = False
        self.connection_status = 'not connected'

        self.prev_status = None

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        self.brake_pub = self.create_publisher(Int16, "/brake_status", 10)
        self.nav_active_pub = self.create_publisher(Bool, "/navigation_active", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # self.odom_pub = self.create_publisher(Odometry, "/odometry/agar", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

        self.navigation_active = False

        self.navigation_active_sub  = self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)

        # self.brake_pub = rospy.Publisher("/brake_status", Int16, queue_size=1)
        # self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size = 1)
        # self.odom_pub = rospy.Publisher("odometry/agar_odom", Odometry, queue_size=10)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.nav_active_pub = rospy.Publisher('/navigation_active', Bool, queue_size=1)
        # self.notifications_pub = rospy.Publisher("/agar_notifications", String, queue_size=1)


        # self.cli = self.create_client(GetParameters, '/navsat_transform/get_parameters')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = GetParameters.Request()

        # self.req.names = ['datum']
        # self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # res = self.future.result()
        # self.datum_latitude     = res.values[0].double_array_value[0]
        # self.datum_longitude    = res.values[0].double_array_value[1]
        # self.datum_latitude         = 43.330890405343304
        # self.datum_longitude        = 21.893977363705066
        self.datum_latitude         = 43.33100874425
        self.datum_longitude        = 21.89109394616667

        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_to_device() 

    def connect_to_device(self):
        while(True):
            print('Connection...')
            if self.socket:
                self.socket.close()
                self.socket = None
            
            self.connected = False
            self.connection_status = 'not connected'

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            try:
                self.socket.settimeout(5)
                self.socket.connect(self.address)
                self.socket.sendto(b'connect', self.address)

                self.get_logger().info(f'{self.socket}')

                self.connected = True
                self.connection_status = 'connected'
                self.get_logger().info('Conection status: {0}'.format(self.connection_status))
                self.timer_gps.reset()
                return
            except socket.timeout:
                self.connection_status = 'connect timeout'
            except socket.error as msg:
                self.connection_status = 'connect error ({0})'.format(msg)

            self.get_logger().info('Conection status: {0}'.format(self.connection_status))
            self.get_logger().info('Reconnectiong in 3 seconds...')

            time.sleep(3) 

    def __del__(self):
        print('Closing connection...')
        if self.socket:
            self.socket.close()

    def read_gps(self):
        try:
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))

            # (data_01, addr) = self.socket.recvfrom(1024)
            # if '$GNGGAH' in data_01.decode():
            #     (data_01, addr) = self.socket.recvfrom(1024)
            # (data_02, addr) = self.socket.recvfrom(1024)

            (data_02, addr) = self.socket.recvfrom(1024)
            if '$GNGGAH' in data_02.decode():
                (data_02, addr) = self.socket.recvfrom(1024)
            (data_01, addr) = self.socket.recvfrom(1024)

            # self.get_logger().info('Data 01: {0}'.format(data_01))
            # self.get_logger().info('Data 01: {0}'.format(data_02))

            # data_01 = b'$GNGGA,130355.50,4319.86211564,N,02153.46818735,E,4,12,1.4,196.7865,M,45.0941,M,1.5,0*62\r\n'
            # data_02 = b'$GNGGAH,130355.50,4319.86146616,N,02153.46712102,E,4,09,2.5,198.0816,M,45.0941,M,1.5,0*2F\r\n'

            # self.get_logger().info('Data 01: {0}'.format(data_01))
            # self.get_logger().info('Data 01: {0}'.format(data_02))

            if data_01 == '' or data_02 == '':
                print('Lost connection. Trying to reconnect...')
                # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
                self.connect_to_device()
            else:
                self.parse_data(data_01, data_02)
                self.connection_status = 'receiving GPS messages'
        except socket.timeout as t:
            self.connection_status = 'no GPS messages received'
            # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
        except socket.error:
            # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
            pass

    def parse_data(self, data_01, data_02):

        gps_data_1 = gpgga.arr2data(data_02)
        gps_data_2 = gpgga.arr2data(data_01)

        # self.get_logger().info('GPS Data 1: lat: {0}, lng: {1}'.format(gps_data_1.latitude, gps_data_1.longitude))
        # self.get_logger().info('GPS Data 2: lat: {0}, lng: {1}'.format(gps_data_2.latitude, gps_data_2.longitude))

        gps_status = self.get_gps_status(gps_data_1.gps_status, gps_data_2.gps_status)
        # self.get_logger().info('gps_status: {0}'.format(gps_status))

        if gps_status != self.prev_status:
            status_name = self.get_gps_name(gps_status)
            # self.send_notifications('Current GPS Status: {0}'.format(status_name))
            self.get_logger().info('Current GPS Status: {0}'.format(status_name))
            self.prev_status = gps_status

        # if self.navigation_active and gps_status != 4:
        #     self.brake_robot()
        #     self.publish_odometry((0, 0), tf.transformations.quaternion_from_euler(0, 0, 0))
        #     return
        
        self.publish_data(gps_data_1, gps_data_2, gps_status)

    def publish_data(self, gps_data_1, gps_data_2, gps_status):
        
        position_1 = alvinxy.ll2xy(gps_data_1.latitude, gps_data_1.longitude, self.datum_latitude, self.datum_longitude)
        position_2 = alvinxy.ll2xy(gps_data_2.latitude, gps_data_2.longitude, self.datum_latitude, self.datum_longitude)

        mid_x = (position_1[0] + position_2[0]) / 2
        mid_y = (position_1[1] + position_2[1]) / 2

        (mid_lat, mid_lng) = alvinxy.xy2ll(mid_x, mid_y, self.datum_latitude, self.datum_longitude)
        mid_alt = int((gps_data_1.altitude + gps_data_2.altitude) / 2)

        position = (mid_x, mid_y)

        # self.get_logger().info('Mid position: x: {0}, y: {1}'.format(mid_x, mid_y))
        gps_fix = alvinxy.xy2ll(position[0], position[1], self.datum_latitude, self.datum_longitude)
        # self.get_logger().info('GPS Fix: lat: {0}, lng: {1}'.format(gps_fix[0], gps_fix[1]))


        heading = atan2(position_2[1] - position_1[1], position_2[0] - position_1[0]) + (pi / 2)

        # cov = random.randrange(0, 1) / 100

        gps_fix_msg = NavSatFix()
        gps_fix_msg.header.stamp        = self.get_clock().now().to_msg()
        gps_fix_msg.header.frame_id     = "gps_link"
        gps_fix_msg.latitude            = gps_fix[0]
        gps_fix_msg.longitude           = gps_fix[1]
        gps_fix_msg.altitude            = float(mid_alt)
        gps_fix_msg.status.status       = 0
        gps_fix_msg.status.service      = 1
        # gps_fix_msg.position_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        gps_fix_msg.position_covariance_type = 2
        self.gps_pub.publish(gps_fix_msg)

        # self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))

        # heading = random.randrange(0, 100) / 100
        orientation = quaternion_from_euler(0, 0, heading)
        # self.get_logger().info('Orientation: {0}'.format(orientation))

        imu_msg = Imu()
        imu_msg.header.stamp        = self.get_clock().now().to_msg()
        imu_msg.header.frame_id     = "imu"
        imu_msg.orientation.x         = orientation[0]
        imu_msg.orientation.y         = orientation[1]
        imu_msg.orientation.z         = orientation[2]
        imu_msg.orientation.w         = orientation[3]
        # imu_msg.orientation_covariance  = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        # imu_msg.angular_velocity_covariance  = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        # imu_msg.angular_velocity_covariance  = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.imu_pub.publish(imu_msg)

        # self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))


        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = orientation[0]
        odom_msg.pose.pose.orientation.y = orientation[1]
        odom_msg.pose.pose.orientation.z = orientation[2]
        odom_msg.pose.pose.orientation.w = orientation[3]
        self.odom_pub.publish(odom_msg)
        
        self.tf_broadcaster.sendTransform(self.set_transform('odom', 'base_footprint', [position[0], position[1], 0.0], orientation))
        self.tf_broadcaster.sendTransform(self.set_transform('map', 'odom', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        # self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))


    def get_gps_name(self, gps_status):
        if gps_status == 1:
            return 'Single point positioning'
        elif gps_status == 2:
            return 'SBAS differential positioning'
        elif gps_status == 4:
            return 'RTK fixed solution'
        elif gps_status == 5:
            return 'RTK floating point'
        elif gps_status == 6:
            return 'Inertial navigation positioning'
        else:
            return 'No positioning'

    def get_gps_status(self, status_1, status_2):
        if status_1 == status_2:
            return status_1
        elif status_1 == 4 or status_2 == 4:
            return 4
        elif status_1 == 0 or status_2 == 0:
            return 0
        elif status_1 == 1 or status_2 == 1:
            return 1
        elif status_1 == 2 or status_2 == 2:
            return 2
        elif status_1 == 5 or status_2 == 5:
            return 5
        elif status_1 == 6 or status_2 == 6:
            return 6
        return 0
    
    def navigation_active_callback(self, msg):
        self.navigation_active  = msg.data


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
    toknav_gps = ToknavGpsDriver()
    rclpy.spin(toknav_gps)
    toknav_gps.destroy_node()
    rclpy.shutdown()

    toknav_gps.get_logger().info('Closed')

if __name__ == '__main__':
    main()