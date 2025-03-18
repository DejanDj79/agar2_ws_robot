# from example_interfaces.srv import AddTwoInts

import rclpy
import os
import csv

from rclpy.node import Node
from agar_msgs.srv import GetMissionEditor
from agar_msgs.msg import LatLng
from agar_common.agar_common_library import agar_navigation


class MissionEditorGet(Node):

    def __init__(self):
        super().__init__('get_mission_editor_service')
        self.srv = self.create_service(GetMissionEditor, 'agar/get_mission_editor_service', self.service_callback)

    def service_callback(self, request, response):
        
        try:
            name = request.filename
            waypoints_path = '/home/nikolica/ros_data/waypoints/'
            mission_file = waypoints_path + name

            response_data = []

            if os.path.exists(mission_file):
                with open(mission_file, 'r') as file:
                    reader = csv.DictReader(file, delimiter = '|', fieldnames=agar_navigation.csv_header)
                    data = list(reader)
                    waypoints = [agar_navigation.waypoint_csv(i) for i in data]

                    i = 0
                    while i < len(waypoints):
                        msg = LatLng()
                        msg.latitude    = waypoints[i].latitude
                        msg.longitude   = waypoints[i].longitude
                        response_data.append(msg)
                        i += 1

            response.positions = response_data
        except Exception as exc:
            self.get_logger().error('Error: %r', exc)
            response.positions = []


        return response

def main():
    rclpy.init()

    service = MissionEditorGet()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()