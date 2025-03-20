# from example_interfaces.srv import AddTwoInts

import rclpy
import os
import csv

from rclpy.node import Node
from agar_msgs.srv import AgarWaypoints
from agar_msgs.msg import AgarWaypoint as AgarWaypointMsg


class WaypointService(Node):

    def __init__(self):
        super().__init__('agar_waypoint_service')
        self.srv = self.create_service(AgarWaypoints, 'agar_waypoint_service', self.waypoint_callback)

    def waypoint_callback(self, request, response):

        try:
            csv_header = ['latitude', 'longitude', 'reverse', 'reverse_turn', 'tank', 'speed', 'agar_action', 'combo_value', 'combo_text', 'param_1', 'param_2', 'param_3', 'param_4', 'visited']
            waypoints_path = '/home/nikolica/ros_data/waypoints/'
            waypoint_filename = request.waypoint_file_name

            waypoint_list = []

            self.get_logger().info(f'waypoint_filename: {waypoints_path + waypoint_filename}')

            file_exists = False if waypoint_filename == 'None' else os.path.exists(waypoints_path + waypoint_filename)
            self.get_logger().info(f'File exists: {file_exists}')
            if file_exists:
                with open(waypoints_path + waypoint_filename, 'r') as file:
                    reader = csv.DictReader(file, delimiter = '|',
                        fieldnames=csv_header)
                    data = list(reader)

                    self.get_logger().info(f'data: {len(data)}')

                    for wp in data:
                        waypoint = AgarWaypointMsg()
                        waypoint.latitude               = float(wp.get('latitude'))
                        waypoint.longitude              = float(wp.get('longitude'))
                        waypoint.reverse                = True if wp.get('reverse').lower() == 'true' else False
                        waypoint.reverse_turn           = True if wp.get('reverse_turn').lower() == 'true' else False
                        waypoint.tank                   = True if wp.get('tank').lower() == 'true' else False
                        waypoint.speed                  = float(wp.get('speed'))
                        waypoint.agar_action            = int(wp.get('agar_action'))
                        waypoint.combo_value            = int(wp.get('combo_value'))
                        waypoint.combo_text             = wp.get('combo_text')
                        waypoint.param_1                = int(wp.get('param_1'))
                        waypoint.param_2                = int(wp.get('param_2'))
                        waypoint.param_3                = int(wp.get('param_3'))
                        waypoint.param_4                = int(wp.get('param_4'))
                        waypoint.visited                = True if wp.get('visited').lower() == 'true' else False
                        waypoint_list.append(waypoint)
            response.waypoints = waypoint_list
        except Exception as exc:
            self.get_logger().error('Error: %r', exc)
            response.waypoints = []


        return response

def main():
    rclpy.init()

    service = WaypointService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()