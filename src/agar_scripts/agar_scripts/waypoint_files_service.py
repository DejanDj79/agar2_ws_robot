# from example_interfaces.srv import AddTwoInts

import rclpy
import os

from rclpy.node import Node
from agar_msgs.srv import AgarWaypointFiles


class WaypointFilesService(Node):

    def __init__(self):
        super().__init__('agar_waypoint_files')
        self.srv = self.create_service(AgarWaypointFiles, 'agar_waypoint_files', self.waypoint_files_callback)

    def waypoint_files_callback(self, request, response):
        data = []
        try:
            dir = '/home/nikolica/ros_data/waypoints/'
            for f in os.listdir(dir):
                if f.endswith(".csv"):
                    data.append(f)
            response.files = data
        except Exception as exc:
            self.get_logger().error('Error: %r', exc)
            response.files = []
        return response

def main():
    rclpy.init()

    service = WaypointFilesService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()