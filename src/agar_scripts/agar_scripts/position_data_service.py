import rclpy

from rclpy.node import Node
from agar_msgs.srv import PositionData
from std_msgs.msg import Bool, UInt64, Int16
from sensor_msgs.msg import NavSatFix


class PositionDataService(Node):

    def __init__(self):
        super().__init__('position_data_service')

        self.navigation_active_sub      = self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)
        self.reset_waypoints_sub        = self.create_subscription(UInt64, '/reset_waypoints', self.reset_waypoints_callback, 10)
        self.visited_waypoint_sub       = self.create_subscription(Int16, "/visited_waypoint", self.visited_waypoint_callback, 10)
        self.gps_subscriber             = self.create_subscription(NavSatFix, 'gps/fix', self.gps_fix_callback, 10)

        self.init_data()

        self.srv = self.create_service(PositionData, 'position_data_service', self.position_data_callback)

    def init_data(self):
        self.altitude = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_status = 0
        self.heading = 0.0
        self.navigation_active = False
        self.mission = ""
        self.current_waypoint = -1
        self.current_mission = 0

    def navigation_active_callback(self, msg):
        self.navigation_active = msg.data

    def reset_waypoints_callback(self, data):
        self.init_data()

    def visited_waypoint_callback(self, data):
        self.current_waypoint = data

    def gps_fix_callback(self, msg):
        self.gps_status     = msg.status.status
        self.latitude       = msg.latitude
        self.longitude      = msg.longitude
        self.altitude       = msg.altitude

    def position_data_callback(self, request, response):
        data = []
        try:
            response.altitude           = self.altitude
            response.current_mission    = self.current_mission
            response.current_waypoint   = self.current_waypoint
            response.gps_status         = self.gps_status
            response.heading            = self.heading
            response.latitude           = self.latitude
            response.longitude          = self.longitude
            response.mission            = self.mission
            response.navigation_active  = self.navigation_active

        except Exception as exc:
            self.get_logger().error('Error: %r', exc)

        return response

def main():
    rclpy.init()

    service = PositionDataService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()