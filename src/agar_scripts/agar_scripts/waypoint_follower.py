import rclpy
import os
import csv
import numpy as np

from math import degrees, atan2, sqrt, acos, sin, cos
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.node import Node

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Bool, String
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Path 
from agar_msgs.msg import CollisionZones
from rcl_interfaces.srv import GetParameters


# import agar_common.agar_common_library.agar_waypoint
from agar_common.agar_common_library import agar_waypoint
from agar_common.agar_common_library.alvinxy import ll2xy


from random import randrange
from enum import Enum

class robot_action(Enum):
    FOLLOW_PATH     = 1
    STOP            = 2
    SNAKE           = 3
    REVERSE         = 4
    SPIN            = 5
    BACKUP          = 6
    NO_ACTION       = 7


class RobotPosition(object):
    x       = 0.0
    y       = 0.0
    z       = 0.0
    deg     = 0.0


class WaypointFollower(Node):

    #region Initialization
    def __init__(self):
        super().__init__('waypoint_follower')

        self.get_logger().info('Initialization completed')

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.run)
        # self.timer = self.create_timer(0.01, self.publish_waypoint)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_drive_base_controller/cmd_vel_unstamped', 10)
        self.waypoints_path_pub = self.create_publisher(Path, "/plan", 10)
        self.angle_path_pub     = self.create_publisher(Path, "/angle_path", 10)

        self.collision_zones_sub    = self.create_subscription(CollisionZones, 'collision_zones', self.collision_zones_callback, 10)
        self.navigation_active_sub  = self.create_subscription(Bool, 'navigation_active', self.navigation_active_callback, 10)
        self.load_waypoints_sub     = self.create_subscription(String, '/load_waypoints', self.load_waypoints_callback, 10)

        self.source_frame = 'odom'
        self.target_frame = 'base_footprint'

        self.robot_position = RobotPosition()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.is_slow    = False

        self.current_action = robot_action.NO_ACTION
        self.backup_distance = 0.0

        self.waypoint_index         = 0
        self.current_waypoint       = None
        self.waypoints              = []
        self.waypoints_csv          = None
        self.waypoint_file_name     = None
        self.waypoints_path         = '/home/nikolica/ros_data/waypoints/'

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.cli = self.create_client(GetParameters, '/navsat_transform/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()

        self.req.names = ['datum']
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()
        self.datum_latitude     = res.values[0].double_array_value[0]
        self.datum_longitude    = res.values[0].double_array_value[1]


        # self.waypoints.append(agar_waypoint.waypoint(0, 0, 0, 10, 0))
        # self.waypoints.append(agar_waypoint.waypoint(1, 0, 0, 8.66, 5))
        # self.waypoints.append(agar_waypoint.waypoint(2, 0, 0, 5, 8.66))
        # self.waypoints.append(agar_waypoint.waypoint(3, 0, 0, 0, 10))
        # self.waypoints.append(agar_waypoint.waypoint(4, 0, 0, -5, 8.66))
        # self.waypoints.append(agar_waypoint.waypoint(5, 0, 0, -8.66, 5))
        # self.waypoints.append(agar_waypoint.waypoint(6, 0, 0, -10, 0))
        # self.waypoints.append(agar_waypoint.waypoint(7, 0, 0, -8.66, -5))
        # self.waypoints.append(agar_waypoint.waypoint(8, 0, 0, -5, -8.66))
        # self.waypoints.append(agar_waypoint.waypoint(9, 0, 0, 0, -10))
        # self.waypoints.append(agar_waypoint.waypoint(10, 0, 0, 5, -8.66))
        # self.waypoints.append(agar_waypoint.waypoint(11, 0, 0, 8.66, -5))

        # self.timer = self.create_timer(1, self.change_waypoint)

    def change_waypoint(self):
        self.waypoint_index += 1
        if self.waypoint_index > len(self.waypoints) - 1:
            self.waypoint_index = 0
        self.current_waypoint = self.waypoints[self.waypoint_index]

        self.current_action = robot_action.FOLLOW_PATH
    #endregion

    #region Load waypoints
    def load_waypoints_from_file(self):
        try:
            with open(self.waypoints_path + self.waypoint_file_name, 'r') as file:
                reader = csv.DictReader(file, delimiter = '|', fieldnames=agar_waypoint.csv_header)
                data = list(reader)
                self.waypoints_csv = [agar_waypoint.waypoint_csv(i) for i in data]
                self.generate_waypoint_list()
        except Exception as ex:
            self.get_logger().error(ex)

    def generate_waypoint_list(self):

        # (pos_x, pos_y) = ll2xy(self.waypoints[0].latitude, self.waypoints[0].longitude, self.datum_latitude, self.datum_longitude)
        # self.waypoint.x = pos_x
        # self.waypoint.y = pos_y
        # self.publish_waypoint()
        # self.current_action = robot_action.FOLLOW_PATH

        self.waypoints = []
        self.path = Path()
        self.path.header.frame_id = 'map'

        pose = PoseStamped()
        pose.pose.position.x = self.robot_position.x
        pose.pose.position.y = self.robot_position.y
        # pose.pose.position.z = 0.0
        self.path.poses.append(pose)
        
        i = 0
        start_x = self.robot_position.x
        start_y = self.robot_position.y
        while i < len(self.waypoints_csv):
            (pos_x, pos_y) = ll2xy(self.waypoints_csv[i].latitude, self.waypoints_csv[i].longitude, self.datum_latitude, self.datum_longitude)

            pose = PoseStamped()
            pose.pose.position.x = pos_x
            pose.pose.position.y = pos_y
            self.path.poses.append(pose)
            wp = agar_waypoint.waypoint(i, start_x, start_y, pos_x, pos_y)
            self.waypoints.append(wp)

            if i == 0: self.current_waypoint = wp

            start_x = pos_x
            start_y = pos_y
            i += 1

        # self.path.poses.pop(0)
        if (len(self.waypoints) > 0):
            # self.get_logger().info(f'{self.waypoints}')
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.waypoints_path_pub.publish(self.path)
    #endregion

    #region Subscriber callback
    def collision_zones_callback(self, msg):

        if self.is_slow == False and msg.front_slow == True:
            self.get_logger().warn("Collision ahead. Entering slow speed mode.")
        elif self.is_slow == True and msg.front_slow == False:
            self.get_logger().warn("Exiting slow speed mode.")

        self.is_slow = msg.front_slow

        # if self.current_action == robot_action.FOLLOW_PATH and msg.front_stop == True:
        #     self.get_logger().warn("Collision ahead. Activating stop and backup.")
        #     trans = self.tf_buffer.lookup_transform('base_footprint', f'waypoint_{self.waypoint_index}', Time())
        #     distance_in_m = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        #     self.backup_distance = distance_in_m + 1.0
        #     self.current_action = robot_action.BACKUP

    def navigation_active_callback(self, msg):
        if msg.data is True:
            self.current_action = robot_action.FOLLOW_PATH
        else:
            self.current_action = robot_action.STOP

    def load_waypoints_callback(self, msg):
        self.waypoint_file_name = msg.data
        if os.path.exists(self.waypoints_path + self.waypoint_file_name):
            self.load_waypoints_from_file()
        else:
            self.get_logger().warn(f'The path {self.waypoints_path}{self.waypoint_file_name} does not exists')
    #endregion

    #region Publisher
    def publish_pose_path(self, point):
        msg_path = Path()
        msg_path.header.frame_id = "map"
        msg_path.header.stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.pose.position.x = self.robot_position.x
        pose.pose.position.y = self.robot_position.y
        msg_path.poses.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        msg_path.poses.append(pose)

        self.angle_path_pub.publish(msg_path) 
    #endregion

    #region Path following
    def run(self):
        self.get_robot_position()

        if self.current_action == robot_action.FOLLOW_PATH:
            self.follow_path()
        if self.current_action == robot_action.BACKUP:
            self.backup()
            
        # self.log()

    def follow_path(self):
        try:
            # self.get_logger().info(f'waypoint start: ({self.current_waypoint.start_x}, {self.current_waypoint.start_y}), end: ({self.current_waypoint.end_x}, {self.current_waypoint.end_y})')

            start_point     = (self.current_waypoint.start_x, self.current_waypoint.start_y)
            end_point       = (self.current_waypoint.end_x, self.current_waypoint.end_y)

            angle_deg       = self.calculate_angle(start_point, end_point)
            distance_in_m   = sqrt(pow(self.robot_position.x - end_point[0], 2) + pow(self.robot_position.y - end_point[1], 2))

            # (point_short, point_long) = self.point_projection(start_point, end_point, 1, 8)
            # angle_short_deg         = self.calculate_angle(start_point, point_short)
            # angle_long_deg          = self.calculate_angle(start_point, point_long)

            # angle_deg = angle_short_deg
            # if angle_long_deg < 1:
            #     angle_deg = angle_long_deg
            #     self.publish_pose_path(point_long)
            # else:
            #     self.publish_pose_path(point_short)
            self.publish_pose_path(end_point)
            
            self.get_logger().info(f'angle: {round(angle_deg, 2)}, distance: {round(distance_in_m, 2)}, waypoint: {round(self.waypoint_index, 2)}, ({round(self.current_waypoint.end_x, 2)}, {round(self.current_waypoint.end_y, 2)})')

            if abs(angle_deg) > 30: speed_z = 1.0
            elif abs(angle_deg) > 10 and abs(angle_deg) <= 30: speed_z = 0.5
            elif abs(angle_deg) > 2 and abs(angle_deg) <= 10: speed_z = 0.25
            elif abs(angle_deg) > 1 and abs(angle_deg) <= 2: speed_z = 0.1
            else: speed_z = 0.0

            if distance_in_m < 2.0:
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    self.get_logger().info(f'Moving to waypoint {self.waypoint_index}')
                    self.current_waypoint = self.waypoints[self.waypoint_index]
                else:
                    self.get_logger().info('All waypoints visited')
                    self.current_action = robot_action.STOP
                # self.backup_distance = distance_in_m + 0.5
                # self.current_action = robot_action.STOP
            
            if self.is_slow:
                speed_x = 0.3
            else:
                speed_x = 1.0

            if speed_z > 1: speed_z = 1.0
            if angle_deg < 0: speed_z *= -1.0

            msg = Twist()
            # msg.header.stamp = self.get_clock().now().to_msg()
            msg.linear.x = speed_x
            msg.angular.z = speed_z
            self.cmd_vel_pub.publish(msg)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to waypoint_01: {ex}')

    def backup(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', f'waypoint_{self.waypoint_index}', Time())
            distance_in_m = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            angle = atan2(trans.transform.translation.y, trans.transform.translation.x)
            angle_deg = degrees(angle)
            # self.get_logger().info(f'Distance: {round(distance_in_m, 2)}, angle waypoint: {round(angle, 2)}, {round(angle_deg, 2)}')
            
            if distance_in_m > self.backup_distance:
                self.get_logger().warn("Backup completed. Perform path following.")
                self.backup_distance = 0
                self.current_action = robot_action.FOLLOW_PATH

            msg = Twist()
            msg.linear.x = -0.3
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to waypoint_{self.waypoint_index}: {ex}')
    #endregion

    #region Angle and point calculation
    def calculate_angle(self, start_point, end_point):
        v_A1_x = self.robot_position.x
        v_A1_y = self.robot_position.y
        v_A2_x = v_A1_x + (0.3 * cos(self.robot_position.z))
        v_A2_y = v_A1_y + (0.3 * sin(self.robot_position.z))

        lineA = [[v_A1_x, v_A1_y], [v_A2_x, v_A2_y]]
        lineB = [[end_point[0], end_point[1]], [start_point[0], start_point[1]]]

        vA = [(lineA[0][0]-lineA[1][0]), (lineA[0][1]-lineA[1][1])]
        vB = [(lineB[0][0]-lineB[1][0]), (lineB[0][1]-lineB[1][1])]
        
        cross = np.cross(vB, vA)
        
        dot_prod = np.dot(vA, vB)
        magA = np.dot(vA, vA)**0.5
        magB = np.dot(vB, vB)**0.5
        
        temp = dot_prod / magB / magA
        if temp > 1: temp = 1
        elif temp < -1: temp = -1
        angle = acos(temp)
        
        if cross < 0:
            angle *= -1

        ang_deg = 180 - (degrees(angle) % 360)
        if ang_deg - 180 >= 0:
            ang_deg =  360 - ang_deg
        return ang_deg

    def point_projection(self, wp_start, wp_end, dis_short, dis_long):
        vector_AB = (wp_end[0] - wp_start[0], wp_end[1] - wp_start[1])
        vector_AP = (self.robot_position.x - wp_start[0], self.robot_position.y - wp_start[1])
        
        scalar_product = vector_AB[0] * vector_AP[0] + vector_AB[1] * vector_AP[1]
        length_AB = vector_AB[0]**2 + vector_AB[1]**2
        t = scalar_product / length_AB
        
        x_proj = wp_start[0] + t * vector_AB[0]
        y_proj = wp_start[1] + t * vector_AB[1]
        projection = (x_proj, y_proj)
        
        duzina_AB = sqrt(length_AB)
        jedinicni_vektor = (vector_AB[0] / duzina_AB, vector_AB[1] / duzina_AB)
        
        point_short = (projection[0] + dis_short * jedinicni_vektor[0], projection[1] + dis_short * jedinicni_vektor[1])
        point_long = (projection[0] + dis_long * jedinicni_vektor[0], projection[1] + dis_long * jedinicni_vektor[1])


        # vector_AB = np.array([wp_end[0] - wp_start[0], wp_end[1] - wp_start[1]])
        # vector_AP = np.array([self.robot_position.x - wp_start[0], self.robot_position.y - wp_start[1]]) 
        
        # length_AB = vector_AB[0]**2 + vector_AB[1]**2
        # duzina_AB = sqrt(length_AB)
        # jedinicni_vektor = (vector_AB[0] / duzina_AB, vector_AB[1] / duzina_AB)

        # vector_AP_norm = np.sqrt(sum(vector_AP**2))
        # move = (np.dot(vector_AB, vector_AP) / vector_AP_norm**2) * vector_AP
        # projection = [wp_end[0] + move[0], wp_end[1] + move[1]]
        
        # point_short = (projection[0] + dis_short * jedinicni_vektor[0], projection[1] + dis_short * jedinicni_vektor[1])
        # point_long = (projection[0] + dis_long * jedinicni_vektor[0], projection[1] + dis_long * jedinicni_vektor[1])
        
        return (point_short, point_long)
    #endregion

    #region Transforms
    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, Time())
            
            self.robot_position.x = trans.transform.translation.x
            self.robot_position.y = trans.transform.translation.y

            orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            (roll, pitch, self.robot_position.z) = euler_from_quaternion(orientation_list)

            # self.get_logger().info(f'x: {round(self.robot_position.x), 2}, y: {round(self.robot_position.y, 2)}, z: {round(self.robot_position.z, 4)}')

            self.robot_position.deg = degrees(self.robot_position.z)
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            
    # def publish_waypoint(self):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'map'
    #     t.child_frame_id = f'waypoint_{self.waypoint_index}'
    #     t.transform.translation.x = self.waypoint.x
    #     t.transform.translation.y = self.waypoint.y
    #     t.transform.translation.z = 0.0
    #     q = quaternion_from_euler(0, 0, 0)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]
    #     # self.tf_broadcaster.sendTransform(t)
    #     self.tf_static_broadcaster.sendTransform(t)
    #endregion

    #region Other
    def log(self):
        self.get_logger().info(
            f'Position x: {round(self.robot_position.x, 2)}, y: {round(self.robot_position.y, 2)}, ' +
                f'yaw: {round(self.robot_position.z, 2)}, degress: {round(self.robot_position.deg, 2)}')

    #endregion

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()
    waypoint_follower.get_logger().info('Closed')

if __name__ == '__main__':
    main()