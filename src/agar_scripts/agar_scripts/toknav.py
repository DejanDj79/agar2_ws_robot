import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_from_euler

import socket
import time
import sys

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu

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

        # self.brake_pub = rospy.Publisher("/brake_status", Int16, queue_size=1)
        # self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size = 1)
        # self.odom_pub = rospy.Publisher("odometry/agar_odom", Odometry, queue_size=10)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.nav_active_pub = rospy.Publisher('/navigation_active', Bool, queue_size=1)
        # self.notifications_pub = rospy.Publisher("/agar_notifications", String, queue_size=1)

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
            (data_01, addr) = self.socket.recvfrom(1024)
            if '$GNGGAH' in data_01.decode():
                (data_01, addr) = self.socket.recvfrom(1024)
            (data_02, addr) = self.socket.recvfrom(1024)

            if data_01 == '' or data_02 == '':
                print('Lost connection. Trying to reconnect...')
                # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
                self.connect_to_device()
            else:
                # self.parse_data(data_01, data_02)
                self.connection_status = 'receiving GPS messages'
        except socket.timeout as t:
            self.connection_status = 'no GPS messages received'
            # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
        except socket.error:
            # self.publish_odometry((0, 0), quaternion_from_euler(0, 0, 0))
            pass




def main(args=None):
    rclpy.init(args=args)
    toknav_gps = ToknavGpsDriver()
    rclpy.spin(toknav_gps)
    toknav_gps.destroy_node()
    rclpy.shutdown()

    toknav_gps.get_logger().info('Closed')

if __name__ == '__main__':
    main()