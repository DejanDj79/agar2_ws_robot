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

class FakeSensors(Node):
    def __init__(self):
        super().__init__('fake_sensors')

        self.rate = 1/20

        self.gps_data = []
        with open('/home/nikolica/Documents/gps_data.txt', 'r') as file:
            # Prolazak kroz svaku liniju u fajlu
            for line in file:
                # Učini nešto sa svakom linijom
                self.gps_data.append(line)

        self.data_count      = len(self.gps_data)
        self.current_count   = 0


        # timer_period = 1.0 / 20 # seconds 20HZ
        # self.timer_gps = self.create_timer(timer_period, self.publish_data)
        # self.timer_gps.cancel()

        timer_period = 1.0 / 50 # seconds
        self.timer_read_data = self.create_timer(timer_period, self.read_data)



        self.transf_br = TransformBroadcaster(self)

        self.using_receiver_epe = False
        self.lon_std_dev = 0
        self.lat_std_dev = 0
        self.alt_std_dev = 0


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


    def read_data(self):
        for i in range(3):
            nmea_string = self.gps_data[self.current_count]

            self.get_logger().info(f'{nmea_string}')
            
            parsed_sentence = self.parse_nmea_sentence(nmea_string)

            if 'GGA'    in parsed_sentence:
                self.gga    = parsed_sentence['GGA']
            elif 'GGAH'   in parsed_sentence:
                self.ggah   = parsed_sentence['GGAH']
            elif 'GST'    in parsed_sentence:
                data = parsed_sentence['GST']
                self.using_receiver_epe = True
                self.lon_std_dev = data['lon_std_dev']
                self.lat_std_dev = data['lat_std_dev']
                self.alt_std_dev = data['alt_std_dev']

            self.current_count += 1
            if self.current_count >= self.data_count:
                self.current_count = 0

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
    


def main(args=None):
    rclpy.init(args=args)
    fake_sensors = FakeSensors()
    rclpy.spin(fake_sensors)
    fake_sensors.destroy_node()
    rclpy.shutdown()

    fake_sensors.get_logger().info('Closed')

if __name__ == '__main__':
    main()