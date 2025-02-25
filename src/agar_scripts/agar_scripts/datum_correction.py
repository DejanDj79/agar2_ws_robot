import rclpy
from rclpy.node import Node

import sys

import rclpy
from rclpy.node import Node
# from rcl_interfaces.srv import GetParameters, SetParameters
# from rcl_interfaces.msg import Parameter, ParameterType

from sensor_msgs.msg import NavSatFix, NavSatStatus
from robot_localization.srv import SetDatum

from geographic_msgs.msg import GeoPose

class DatumCorrection(Node):
    def __init__(self):
        super().__init__('datum_correction')


        self.fix_received   = False

        self.cli_set = self.create_client(SetDatum, '/datum')
        while not self.cli_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetDatum.Request()

        self.gps_subscriber = self.create_subscription(NavSatFix, 'gps/fix', self.gps_subscriber_callback, 10)
        
        self.get_logger().info('Initialized')


    def gps_subscriber_callback(self, msg):
        status = msg.status.status
        if status == NavSatStatus.STATUS_GBAS_FIX and self.fix_received == False:
            lat = msg.latitude
            lng = msg.longitude
            alt = msg.altitude
            res = self.set_datum(lat, lng, alt)
            if res:
                self.fix_received = True
                # self.destroy_node()
                # rclpy.shutdown()
            
    def set_datum(self, latitude, longitude, altitude):
        try:
            geo_pose = GeoPose()
            geo_pose.position.latitude = latitude
            geo_pose.position.longitude = longitude
            geo_pose.position.altitude = altitude

            self.req.geo_pose = geo_pose

            self.future = self.cli_set.call_async(self.req)
            # rclpy.spin_until_future_complete(self, self.future)
            self.future.result()
            self.get_logger().info('Service call completed')
            return True
        except Exception as exc:
            self.get_logger().info('Service call failed %r' % exc)
            
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