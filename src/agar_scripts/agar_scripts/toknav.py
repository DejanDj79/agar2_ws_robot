import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_from_euler, euler_from_quaternion

import socket
import time
import sys
import re
import datetime
import calendar
from math import atan2, pi, radians, degrees, isnan, sin, cos

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist, Quaternion, QuaternionStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
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

        timer_period = 1.0 / 20 # seconds 20HZ
        self.timer_gps = self.create_timer(timer_period, self.publish_data)
        self.timer_gps.cancel()

        timer_period = 1.0 / 50 # seconds
        self.timer_read_data = self.create_timer(timer_period, self.read_data)
        self.timer_read_data.cancel()

        self.transf_br = TransformBroadcaster(self)

        host = "192.168.0.66"
        port = 66
        self.address = (host, port)
        self.socket = None

        self.connected = False
        self.connection_status = 'not connected'

        self.prev_status = None
        self.has_rtk_fix = True

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.gps_left_pub = self.create_publisher(NavSatFix, "/gps/fix/left", 10)
        self.gps_right_pub = self.create_publisher(NavSatFix, "/gps/fix/right", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        # self.heading_pub = self.create_publisher(QuaternionStamped, '/gps/heading', 10)

        self.brake_pub = self.create_publisher(Int16, "/brake_status", 10)
        self.nav_active_pub = self.create_publisher(Bool, "/navigation_active", 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # self.odom_pub = self.create_publisher(Odometry, "/odometry/agar", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

        self.navigation_active = False

        # self.navigation_active_sub  = self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)

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

        self.use_GNSS_time = False
        self.using_receiver_epe = False

        self.position = [0.0, 0.0, 0.0]
        self.orientation = quaternion_from_euler(0, 0, 0)


        # self.heading    = euler_from_quaternion([0.0, 0.0, -0.9986637550641332, -0.051678857584170644])
        # self.gps        = euler_from_quaternion([0.0, 0.0, 0.9997333805281944, -0.023090427836408713])

        # self.get_logger().info(f'Heading: {self.heading[2]} - {degrees(self.heading[2])}, GPS: {self.gps[2]} - {degrees(self.gps[2])}')

        self.gga    = None
        self.ggah   = None

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        self.parse_maps = {
            "GGA": [
                ("fix_type", int, 6),
                ("latitude", self.convert_latitude, 2),
                ("latitude_direction", str, 3),
                ("longitude", self.convert_longitude, 4),
                ("longitude_direction", str, 5),
                ("altitude", self.safe_float, 9),
                ("mean_sea_level", self.safe_float, 11),
                ("hdop", self.safe_float, 8),
                ("num_satellites", self.safe_int, 7),
                ("utc_time", self.convert_time, 1),
            ],
            "GGAH": [
                ("fix_type", int, 6),
                ("latitude", self.convert_latitude, 2),
                ("latitude_direction", str, 3),
                ("longitude", self.convert_longitude, 4),
                ("longitude_direction", str, 5),
                ("altitude", self.safe_float, 9),
                ("mean_sea_level", self.safe_float, 11),
                ("hdop", self.safe_float, 8),
                ("num_satellites", self.safe_int, 7),
                ("utc_time", self.convert_time, 1),
            ],
            "GST": [
                ("utc_time", self.convert_time, 1),
                ("ranges_std_dev", self.safe_float, 2),
                ("semi_major_ellipse_std_dev", self.safe_float, 3),
                ("semi_minor_ellipse_std_dev", self.safe_float, 4),
                ("semi_major_orientation", self.safe_float, 5),
                ("lat_std_dev", self.safe_float, 6),
                ("lon_std_dev", self.safe_float, 7),
                ("alt_std_dev", self.safe_float, 8),
            ],
            "HDT": [
                ("heading", self.safe_float, 1),
            ],
            "VTG": [
                ("true_course", self.convert_deg_to_rads, 1),
                ("speed", self.convert_knots_to_mps, 5)
            ]
        }

        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX, # -1
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX, # -1
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX, # 0
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX, # 1
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX, # 2
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_SBAS_FIX, # 1 #NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_SBAS_FIX, # 1 #NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

        self.connect_to_device()
        
    #region Convert functions
    def safe_float(self, field):
        try:
            return float(field)
        except ValueError:
            return float('NaN')

    def safe_int(self, field):
        try:
            return int(field)
        except ValueError:
            return 0

    def convert_latitude(self, field):
        return self.safe_float(field[0:2]) + self.safe_float(field[2:]) / 60.0

    def convert_longitude(self, field):
        return self.safe_float(field[0:3]) + self.safe_float(field[3:]) / 60.0
    
    def convert_time(self, nmea_utc):
        """
        Extract time info from a NMEA UTC time string and use it to generate a UNIX epoch time.
        Time information (hours, minutes, seconds) is extracted from the given string and augmented
        with the date, which is taken from the current system time on the host computer (i.e. UTC now).
        The date ambiguity is resolved by adding a day to the current date if the host time is more than
        12 hours behind the NMEA time and subtracting a day from the current date if the host time is
        more than 12 hours ahead of the NMEA time.
        Args:
            nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS[.SS] where
                HH is the number of hours [0,24), MM is the number of minutes [0,60),
                and SS[.SS] is the number of seconds [0,60) of the time in UTC.
        Returns:
            tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
            tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
        """
        # If one of the time fields is empty, return NaN seconds
        if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
            return (float('NaN'), float('NaN'))

        # Get current time in UTC for date information
        utc_time = datetime.datetime.utcnow()
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        nanosecs = 0
        # If the seconds includes a decimal portion, convert it to nanoseconds
        if len(nmea_utc) > 7:
            nanosecs = int(nmea_utc[7:]) * pow(10, 9 - len(nmea_utc[7:]))

        # Resolve the ambiguity of day
        day_offset = int((utc_time.hour - hours)/12.0)
        utc_time += datetime.timedelta(day_offset)
        utc_time = utc_time.replace(hour=hours, minute=minutes, second=seconds)

        unix_secs = calendar.timegm(utc_time.timetuple())
        return (unix_secs, nanosecs)

    def convert_time_rmc(self, date_str, time_str):
        """Convert a NMEA RMC date string and time string to UNIX epoch time.

        Args:
            date_str (str): NMEA UTC date string to convert, formatted as DDMMYY.
            nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS.SS where
                HH is the number of hours [0,24), MM is the number of minutes [0,60),
                and SS.SS is the number of seconds [0,60) of the time in UTC.

        Returns:
            tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
            tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
        """
        # If one of the time fields is empty, return NaN seconds
        if not date_str[0:6] or not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
            return (float('NaN'), float('NaN'))

        pc_year = datetime.date.today().year

        # Resolve the ambiguity of century
        """
        example 1: utc_year = 99, pc_year = 2100
        years = 2100 + int((2100 % 100 - 99) / 50.0) = 2099
        example 2: utc_year = 00, pc_year = 2099
        years = 2099 + int((2099 % 100 - 00) / 50.0) = 2100
        """
        utc_year = int(date_str[4:6])
        years = pc_year + int((pc_year % 100 - utc_year) / 50.0)

        months = int(date_str[2:4])
        days = int(date_str[0:2])

        hours = int(time_str[0:2])
        minutes = int(time_str[2:4])
        seconds = int(time_str[4:6])
        nanosecs = int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))

        unix_secs = calendar.timegm((years, months, days, hours, minutes, seconds))
        return (unix_secs, nanosecs)

    def convert_status_flag(self, status_flag):
        if status_flag == "A":
            return True
        elif status_flag == "V":
            return False
        else:
            return False

    def convert_knots_to_mps(self, knots):
        return self.safe_float(knots) * 0.514444444444

    def convert_deg_to_rads(self, degs):
        return radians(self.safe_float(degs))
    #endregion

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
                self.timer_read_data.reset()
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

    def read_data(self):
        try:
            data_1, remote_address = self.socket.recvfrom(1024)
            data_2, remote_address = self.socket.recvfrom(1024)
            data_3, remote_address = self.socket.recvfrom(1024)
            data = data_1 + data_2 + data_3
            data_list = data.decode("ascii").strip().split("\r\n")

            for nmea_string in data_list:
                parsed_sentence = self.parse_nmea_sentence(nmea_string)
                if 'GGA'    in parsed_sentence:
                    self.ggah    = parsed_sentence['GGA']
                if 'GGAH'   in parsed_sentence:
                    self.gga   = parsed_sentence['GGAH']
                if 'GST'    in parsed_sentence:
                    data = parsed_sentence['GST']
                    # Use receiver-provided error estimate if available
                    self.using_receiver_epe = True
                    self.lon_std_dev = data['lon_std_dev']
                    self.lat_std_dev = data['lat_std_dev']
                    self.alt_std_dev = data['alt_std_dev']

        except socket.error as exc:
            self.get_logger().error("Caught exception socket.error during recvfrom: %s" % exc)
            self.timer_gps.cancel()
            self.timer_read_data.cancel()
            self.socket.close()
            self.connect_to_device()
        except Exception as exc:
            self.get_logger().error("Caught exception: %s" % exc)
            self.timer_gps.cancel()
            self.timer_read_data.cancel()
            self.socket.close()
            self.connect_to_device()

        #region backup
        # try:
        #     self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))
        #     self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))



        #     data, remote_address = self.socket.recvfrom(4096)
        #     data_list = data.decode("ascii").strip().split("\n")

        #     # self.get_logger().info(f'{len(data_list)}')
        #     # self.get_logger().info(f'{data_list}')

        #     for nmea_string in data_list:
        #         parsed_sentence = self.parse_nmea_sentence(nmea_string)
        #         # self.get_logger().info(f'{parsed_sentence}')

        #         if 'GGA' in parsed_sentence:
        #             gps_fix_msg = NavSatFix()
        #             gps_fix_msg.header.stamp        = self.get_clock().now().to_msg()
        #             gps_fix_msg.header.frame_id     = "gps_link"

        #             gps_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        #             data = parsed_sentence['GGA']
        #             if self.use_GNSS_time:
        #                 gps_fix_msg.header.stamp = rclpy.time.Time(seconds=data['utc_time'][0], nanoseconds=data['utc_time'][1]).to_msg()

        #             fix_type = data['fix_type']
        #             if not (fix_type in self.gps_qualities):
        #                 fix_type = -1
        #             gps_qual = self.gps_qualities[fix_type]
        #             default_epe = gps_qual[0]
        #             gps_fix_msg.status.status = gps_qual[1]
        #             gps_fix_msg.position_covariance_type = gps_qual[2]
        #             if gps_fix_msg.status.status > 0:
        #                 self.valid_fix = True
        #             else:
        #                 self.valid_fix = False

        #             gps_fix_msg.status.service = NavSatStatus.SERVICE_GPS
        #             latitude = data['latitude']
        #             if data['latitude_direction'] == 'S':
        #                 latitude = -latitude
        #             gps_fix_msg.latitude = latitude

        #             longitude = data['longitude']
        #             if data['longitude_direction'] == 'W':
        #                 longitude = -longitude
        #             gps_fix_msg.longitude = longitude

        #             # Altitude is above ellipsoid, so adjust for mean-sea-level
        #             altitude = data['altitude'] + data['mean_sea_level']
        #             gps_fix_msg.altitude = altitude

        #             position = alvinxy.ll2xy(latitude, longitude, self.datum_latitude, self.datum_longitude)
        #             self.position[0] = position[0]
        #             self.position[1] = position[1]

        #             # use default epe std_dev unless we've received a GST sentence with epes
        #             if not self.using_receiver_epe or isnan(self.lon_std_dev):
        #                 self.lon_std_dev = default_epe
        #             if not self.using_receiver_epe or isnan(self.lat_std_dev):
        #                 self.lat_std_dev = default_epe
        #             if not self.using_receiver_epe or isnan(self.alt_std_dev):
        #                 self.alt_std_dev = default_epe * 2

        #             hdop = data['hdop']
        #             gps_fix_msg.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
        #             gps_fix_msg.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
        #             gps_fix_msg.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

        #             self.gps_pub.publish(gps_fix_msg)
        #         elif 'HDT' in parsed_sentence:
        #             data = parsed_sentence['HDT']
        #             # self.get_logger().info(f'{data}')
        #             # self.get_logger().info("Heading: {0}".format(data['heading']))
        #             heading = radians(data['heading']) - pi
        #             self.orientation = quaternion_from_euler(0, 0, heading)
        #             imu_msg = Imu()
        #             imu_msg.header.stamp        = self.get_clock().now().to_msg()
        #             imu_msg.header.frame_id     = "imu"
        #             imu_msg.orientation.x         = self.orientation[0]
        #             imu_msg.orientation.y         = self.orientation[1]
        #             imu_msg.orientation.z         = self.orientation[2]
        #             imu_msg.orientation.w         = self.orientation[3]
        #             self.imu_pub.publish(imu_msg)

        #             # current_heading = QuaternionStamped()
        #             # current_heading.header.stamp = self.get_clock().now().to_msg()
        #             # current_heading.header.frame_id = "imu"
        #             # current_heading.quaternion.x = self.orientation[0]
        #             # current_heading.quaternion.y = self.orientation[1]
        #             # current_heading.quaternion.z = self.orientation[2]
        #             # current_heading.quaternion.w = self.orientation[3]
        #             # self.heading_pub.publish(current_heading) 

        #         elif 'GST' in parsed_sentence:
        #             data = parsed_sentence['GST']
        #             # Use receiver-provided error estimate if available
        #             self.using_receiver_epe = True
        #             self.lon_std_dev = data['lon_std_dev']
        #             self.lat_std_dev = data['lat_std_dev']
        #             self.alt_std_dev = data['alt_std_dev']


        #         # self.get_logger().info(f'{self.position}')
        #         # odom_msg = Odometry()
        #         # odom_msg.header.stamp = self.get_clock().now().to_msg()
        #         # odom_msg.header.frame_id = "odom"
        #         # odom_msg.child_frame_id = "base_footprint"
        #         # odom_msg.pose.pose.position.x = self.position[0]
        #         # odom_msg.pose.pose.position.y = self.position[1]
        #         # odom_msg.pose.pose.position.z = self.position[2]
        #         # odom_msg.pose.pose.orientation.x = self.orientation[0]
        #         # odom_msg.pose.pose.orientation.y = self.orientation[1]
        #         # odom_msg.pose.pose.orientation.z = self.orientation[2]
        #         # odom_msg.pose.pose.orientation.w = self.orientation[3]
        #         # self.odom_pub.publish(odom_msg)

        #         # self.tf_broadcaster.sendTransform(self.set_transform('odom', 'base_footprint', [self.position[0], self.position[1], self.position[2]], self.orientation))
        #         # self.tf_broadcaster.sendTransform(self.set_transform('map', 'odom', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        # except socket.error as exc:
        #     self.get_logger().error("Caught exception socket.error during recvfrom: %s" % exc)
        #     self.socket.close()
        #endregion

    def publish_data(self):
        try:
            # self.get_logger().info(f'{self.gga}')
            # self.get_logger().info(f'{self.ggah}')
            # self.get_logger().info(f'----------------------------')

            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'gps_link', [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]))
            self.tf_broadcaster.sendTransform(self.set_transform('base_link', 'imu', [0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))

            if self.gga['fix_type'] == 4 and self.ggah['fix_type'] == 4 and self.has_rtk_fix == False:
                self.has_rtk_fix = True
            elif self.has_rtk_fix == True:
                self.has_rtk_fix = False

            gps_fix_msg = NavSatFix()
            gps_fix_msg.header.stamp        = self.get_clock().now().to_msg()
            gps_fix_msg.header.frame_id     = "gps_link"

            gps_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            if self.use_GNSS_time:
                gps_fix_msg.header.stamp = rclpy.time.Time(seconds=self.gga['utc_time'][0], nanoseconds=self.gga['utc_time'][1]).to_msg()

            fix_type = self.gga['fix_type']
            fix_type_h = self.ggah['fix_type']


            if not (fix_type in self.gps_qualities):
                fix_type = -1

            if not (fix_type_h in self.gps_qualities):
                fix_type_h = -1

            gps_qual = self.gps_qualities[fix_type]
            gps_qual_h = self.gps_qualities[fix_type_h]

            if gps_qual_h[1] < gps_qual[1]:
                gps_qual = gps_qual_h

            default_epe = gps_qual[0]
            gps_fix_msg.status.status = gps_qual[1]
            gps_fix_msg.position_covariance_type = gps_qual[2]

            if gps_fix_msg.status.status > 1: #0:
                self.valid_fix = True
            else:
                self.valid_fix = False
                nav_active = Bool()
                nav_active.data = False
                self.nav_active_pub.publish(nav_active)
                
            gps_fix_msg.status.service = NavSatStatus.SERVICE_GPS

            latitude = (self.gga['latitude'] + self.ggah['latitude']) / 2
            longitude = (self.gga['longitude'] + self.ggah['longitude']) / 2 

            if self.gga['latitude_direction'] == 'S':
                latitude = -latitude
            gps_fix_msg.latitude = latitude

            if self.gga['longitude_direction'] == 'W':
                longitude = -longitude
            gps_fix_msg.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = self.gga['altitude'] + self.gga['mean_sea_level']
            gps_fix_msg.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            # self.get_logger().info(f'Standard deviation: {self.lon_std_dev} - {self.lat_std_dev} - {self.alt_std_dev}')

            hdop = self.gga['hdop']
            gps_fix_msg.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            gps_fix_msg.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            gps_fix_msg.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.gps_pub.publish(gps_fix_msg)

            gps_fix_msg.latitude = self.gga['latitude']
            gps_fix_msg.longitude = self.gga['longitude']
            self.gps_left_pub.publish(gps_fix_msg)

            gps_fix_msg.latitude = self.ggah['latitude']
            gps_fix_msg.longitude = self.ggah['longitude']
            self.gps_right_pub.publish(gps_fix_msg)

            # heading = self.calculate_heading(self.gga['latitude'], self.gga['longitude'], self.ggah['latitude'], self.ggah['longitude'])

            position_1 = alvinxy.ll2xy(self.gga['latitude'], self.gga['longitude'], latitude, longitude)
            position_2 = alvinxy.ll2xy(self.ggah['latitude'], self.ggah['longitude'], latitude, longitude)
            heading = atan2(position_2[1] - position_1[1], position_2[0] - position_1[0]) + (pi / 2)

            self.orientation = quaternion_from_euler(0, 0, heading)
            imu_msg = Imu()
            imu_msg.header.stamp        = self.get_clock().now().to_msg()
            imu_msg.header.frame_id     = "imu"
            imu_msg.orientation.x         = self.orientation[0]
            imu_msg.orientation.y         = self.orientation[1]
            imu_msg.orientation.z         = self.orientation[2]
            imu_msg.orientation.w         = self.orientation[3]
            self.imu_pub.publish(imu_msg)

        except Exception as exc:
            self.get_logger().error("Caught exception: %s" % exc)

    # def calculate_heading(self, lat1, lon1, lat2, lon2):
    #     lat1 = radians(lat1)
    #     lon1 = radians(lon1)
    #     lat2 = radians(lat2)
    #     lon2 = radians(lon2)

    #     dLon = lon2 - lon1
    #     x = sin(dLon) * cos(lat2)
    #     y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dLon))

    #     initial_heading = atan2(x, y) + (pi / 2)
    #     # initial_heading = degrees(initial_heading)
    #     # compass_heading = (initial_heading + 360) % 360  # pretvorimo u kompasni smer

    #     # return radians(compass_heading)
    #     return initial_heading
        
    def parse_nmea_sentence(self, nmea_sentence):
        # Check for a valid nmea sentence

        if not re.match(r'(^\$GP|^\$GN|^\$GL|^\$IN).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
            self.get_logger().error("Regex didn't match, sentence not valid NMEA? Sentence was: %s"
                        % repr(nmea_sentence))
            return False
        fields = [field.strip(',') for field in nmea_sentence.replace('*', ',0*').split(',')]

        # Ignore the $ and talker ID portions (e.g. GP)
        sentence_type = fields[0][3:]

        if sentence_type not in self.parse_maps:
            self.get_logger().error("Sentence type %s not in parse map, ignoring."
                        % repr(sentence_type))
            return False

        parse_map = self.parse_maps[sentence_type]

        parsed_sentence = {}
        for entry in parse_map:
            parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

        if sentence_type == "RMC":
            parsed_sentence["utc_time"] = self.convert_time_rmc(fields[9], fields[1])

        return {sentence_type: parsed_sentence}

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