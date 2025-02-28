import rclpy
from rclpy.node import Node

# import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from robot_localization.srv import SetDatum

from geographic_msgs.msg import GeoPose
from agar_msgs.srv import Datum

class DatumCorrection(Node):
    def __init__(self):
        super().__init__('datum_correction')
        
        self.geo_pose = GeoPose()

        # self.srv = self.create_service(Datum, 'datum_get', self.get_datum_callback)

        self.fix_received   = False

        self.cli_set = self.create_client(SetDatum, '/datum')
        while not self.cli_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /datum not available, waiting again...')
        self.req = SetDatum.Request()

        self.waypoint_datum_set = self.create_client(Datum, '/waypoint_datum')
        while not self.waypoint_datum_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /waypoint_datum not available, waiting again...')
        self.waypoint_datum_req = Datum.Request()

        self.gps_subscriber = self.create_subscription(NavSatFix, 'gps/fix', self.gps_subscriber_callback, 10)
        
        self.get_logger().info('Initialized')


    # def get_datum_callback(self, request, response):
    #     self.get_logger().info(f'Datum: {self.geo_pose.position.latitude} {self.geo_pose.position.longitude} {self.geo_pose.position.altitude}')
    #     response.datum = self.geo_pose
    #     return response

    def gps_subscriber_callback(self, msg):
        status = msg.status.status
        if status == NavSatStatus.STATUS_FIX and self.fix_received == False:
        # if status == NavSatStatus.STATUS_GBAS_FIX and self.fix_received == False:
            lat = msg.latitude
            lng = msg.longitude
            alt = msg.altitude
            self.get_logger().info(f'Received GPS fix: {lat} {lng} {alt}')
            res = self.set_datum(lat, lng, alt)
            if res:
                try:
                    self.waypoint_datum_req.datum = self.geo_pose
                    future = self.waypoint_datum_set.call_async(self.waypoint_datum_req)
                    future.result()
                    self.get_logger().info('Service /waypoint_datum call completed')
                    # self.destroy_node()
                    # rclpy.shutdown()
                    self.fix_received = True
                except Exception as exc:
                    self.get_logger().info('Service /waypoint_datum call failed %r' % exc)
            
    def set_datum(self, latitude, longitude, altitude):
        try:
            self.geo_pose.position.latitude = latitude
            self.geo_pose.position.longitude = longitude
            self.geo_pose.position.altitude = altitude

            self.req.geo_pose = self.geo_pose

            future = self.cli_set.call_async(self.req)
            # rclpy.spin_until_future_complete(self, self.future)
            future.result()
            self.get_logger().info('Service /datum call completed')
            return True
        except Exception as exc:
            self.get_logger().info('Service /datum call failed %r' % exc)
            
        return False
            

def main(args=None):
    rclpy.init(args=args)
    datum_correction = DatumCorrection()
    rclpy.spin(datum_correction)
    # datum_correction.set_datum(43.33057851631734, 21.89032155644895, 212.0)

    datum_correction.destroy_node()
    rclpy.shutdown()

    datum_correction.get_logger().info('Closed')

if __name__ == '__main__':
    main()