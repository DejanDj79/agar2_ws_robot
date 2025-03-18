# from example_interfaces.srv import AddTwoInts

import rclpy
import os
import csv

from rclpy.node import Node
from agar_msgs.srv import SetMissionEditor


class MissionEditorSet(Node):

    def __init__(self):
        super().__init__('set_mission_editor_service')
        self.srv = self.create_service(SetMissionEditor, 'agar/set_mission_editor_service', self.service_callback)

    def service_callback(self, request, response):
        saved = False

        try:
            name = request.filename
            positions = request.positions

            waypoints_path = '/home/nikolica/ros_data/waypoints/'
            mission_file = open(waypoints_path + name + '.csv', "w")
            mission_file_wp = open(waypoints_path + name + '.waypoints', "w")

            mission_file_wp.write('QGC WPL 110\n')
            mission_file_wp.write('0	1	0	16	0	0	0	0	43.6228540	22.2604305	207.820000	0	0	0	100	0	0	0	0	0	0	1\n')

            for pos in positions:
                mission_file.write('{0}|{1}|False|False|False|1.00|0|0|none|0|0|0|0|False\n'.format(pos.latitude, pos.longitude))
                mission_file_wp.write('1	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	{0}	{1}	100.000000	0	0	0	100	0	0	0	0	0	0	1\n'.format(pos.latitude, pos.longitude))
                
            mission_file.close()
            mission_file_wp.close()
            saved = True
        except Exception as exc:
            self.get_logger().error('Error: %r', exc)
            saved = False
            
        response.saved = saved
        return response

def main():
    rclpy.init()

    service = MissionEditorSet()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()