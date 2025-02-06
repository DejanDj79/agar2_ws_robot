import rclpy
import os
import csv
import numpy as np

from math import degrees, atan2, sqrt, pi
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.node import Node

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path 
from agar_msgs.msg import CollisionZones
from rcl_interfaces.srv import GetParameters

from agar_common.agar_common_library import agar_navigation
from agar_common.agar_common_library.alvinxy import ll2xy

from enum import Enum

class robot_action(Enum):
    FOLLOW_PATH     = 1
    STOP            = 2
    WAIT_TO_STOP    = 3
    SNAKE           = 4
    REVERSE         = 5
    SPIN            = 6
    BACKUP          = 7
    NO_ACTION       = 8

class robot_direction(Enum):
    FORWARD         = 1
    BACKWARD        = 2

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

        timer_period        = 0.05  # seconds
        self.timer          = self.create_timer(timer_period, self.run)
        self.timer_stop     = self.create_timer(1.0, self.stop_and_wait)
        self.timer_stop.cancel()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_drive_base_controller/cmd_vel_unstamped', 10)
        self.waypoints_path_pub = self.create_publisher(Path, "/plan", 10)
        self.angle_path_pub     = self.create_publisher(Path, "/angle_path", 10)

        self.collision_zones_sub    = self.create_subscription(CollisionZones, '/collision_zones', self.collision_zones_callback, 10)
        self.navigation_active_sub  = self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)
        self.load_waypoints_sub     = self.create_subscription(String, '/load_waypoints', self.load_waypoints_callback, 10)

        self.source_frame = 'odom'
        self.target_frame = 'base_footprint'

        self.robot_position = RobotPosition()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)


        self.navigation_active      = False
        self.current_action         = robot_action.NO_ACTION
        self.backup_distance        = 0.0
        self.is_slow                = False
        self.default_goal_distance  = 2.2
        self.goal_distance          = self.default_goal_distance
        self.reverse                = False
        self.robot_direction        = robot_direction.FORWARD
        self.reverse_turn_distance  = 1.8
        self.path_short_point       = 1.5
        self.path_long_point        = 8.0

        self.waypoint_index         = 0
        self.current_waypoint       = None
        self.waypoints              = []
        self.addition_waypoints     = []
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
    #endregion

    #region Load waypoints
    def reset_waypoints(self):
        self.current_action         = robot_action.NO_ACTION
        self.navigation_active      = False
        self.goal_distance          = self.default_goal_distance
        self.path                   = Path()
        self.path.header.frame_id   = 'map'

        self.waypoint_index         = 0
        self.current_waypoint       = None
        self.waypoints              = []
        self.waypoints_csv          = None
        self.waypoint_file_name     = None
        
    def load_waypoints_from_file(self):
        try:
            with open(self.waypoints_path + self.waypoint_file_name, 'r') as file:
                reader = csv.DictReader(file, delimiter = '|', fieldnames=agar_navigation.csv_header)
                data = list(reader)
                self.waypoints_csv = [agar_navigation.waypoint_csv(i) for i in data]
                self.generate_waypoint_list()
        except Exception as ex:
            self.get_logger().error(ex)

    def generate_waypoint_list(self):
        pose = PoseStamped()
        pose.pose.position.x = self.robot_position.x
        pose.pose.position.y = self.robot_position.y
        self.path.poses.append(pose)
        
        i = 0
        start_x = self.robot_position.x
        start_y = self.robot_position.y
        total_distance = 0
        while i < len(self.waypoints_csv):
            curr_wp = self.waypoints_csv[i]

            (pos_x, pos_y) = ll2xy(curr_wp.latitude, curr_wp.longitude, self.datum_latitude, self.datum_longitude)

            pose = PoseStamped()
            pose.pose.position.x = pos_x
            pose.pose.position.y = pos_y
            self.path.poses.append(pose)

            wp = agar_navigation.waypoint(i, start_x, start_y, pos_x, pos_y, curr_wp.reverse, curr_wp.reverse_turn, curr_wp.tank, 
                curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
            self.waypoints.append(wp)

            total_distance += sqrt(pow(start_x - pos_x, 2) + pow(start_y - pos_y, 2))
            if i == 0: self.current_waypoint = wp

            start_x = pos_x
            start_y = pos_y

            if curr_wp.reverse_turn and i < len(self.waypoints_csv) - 2:
                a_start_x = wp.start_x
                a_start_y = wp.start_y
                a_end_x = wp.end_x
                a_end_y = wp.end_y
                (b_start_x, b_start_y)  = ll2xy(self.waypoints_csv[i + 1].latitude, self.waypoints_csv[i + 1].longitude, self.datum_latitude, self.datum_longitude)
                (b_end_x, b_end_y)      = ll2xy(self.waypoints_csv[i + 2].latitude, self.waypoints_csv[i + 2].longitude, self.datum_latitude, self.datum_longitude)

                wp1 = self.move_point((a_start_x, a_start_y), (a_end_x, a_end_y), self.reverse_turn_distance) # + 1.0
                wp2 = self.move_point((b_end_x, b_end_y), (b_start_x, b_start_y), self.reverse_turn_distance) # + 1.0

                wp3 = self.move_point((wp1[0], wp1[1]), (wp2[0], wp2[1]), self.reverse_turn_distance)
                wp4 = self.move_point((wp2[0], wp2[1]), (wp1[0], wp1[1]), self.reverse_turn_distance)

                wp = agar_navigation.waypoint(i, start_x, start_y, wp1[0], wp1[1], False, curr_wp.reverse_turn, curr_wp.tank, 
                    curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                    curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                self.waypoints.append(wp)

                total_distance += sqrt(pow(start_x - wp1[0], 2) + pow(start_y - wp1[1], 2))

                pose = PoseStamped()
                pose.pose.position.x = wp1[0]
                pose.pose.position.y = wp1[1]
                self.path.poses.append(pose)

                wp = agar_navigation.waypoint(i, wp1[0], wp1[1], wp3[0], wp3[1], False, curr_wp.reverse_turn, curr_wp.tank, 
                    curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                    curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                self.waypoints.append(wp)

                total_distance += sqrt(pow(wp1[0] - wp3[0], 2) + pow(wp1[1] - wp3[1], 2))

                pose = PoseStamped()
                pose.pose.position.x = wp3[0]
                pose.pose.position.y = wp3[1]
                self.path.poses.append(pose)

                wp = agar_navigation.waypoint(i, wp3[0], wp3[1], wp4[0], wp4[1], True, curr_wp.reverse_turn, curr_wp.tank, 
                    curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                    curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                self.waypoints.append(wp)

                total_distance += sqrt(pow(wp3[0] - wp4[0], 2) + pow(wp3[1] - wp4[1], 2))

                pose = PoseStamped()
                pose.pose.position.x = wp4[0]
                pose.pose.position.y = wp4[1]
                self.path.poses.append(pose)

                wp = agar_navigation.waypoint(i, wp4[0], wp4[1], wp2[0], wp2[1], False, curr_wp.reverse_turn, curr_wp.tank, 
                    curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                    curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                self.waypoints.append(wp)

                total_distance += sqrt(pow(wp4[0] - wp2[0], 2) + pow(wp4[1] - wp2[1], 2))

                pose = PoseStamped()
                pose.pose.position.x = wp2[0]
                pose.pose.position.y = wp2[1]
                self.path.poses.append(pose)

                start_x = wp2[0]
                start_y = wp2[1]

                # wp = agar_navigation.waypoint(i, start_x, start_y, wp3[0], wp3[1], False, curr_wp.reverse_turn, curr_wp.tank, 
                #     curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                #     curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                # self.waypoints.append(wp)

                # pose = PoseStamped()
                # pose.pose.position.x = wp3[0]
                # pose.pose.position.y = wp3[1]
                # self.path.poses.append(pose)

                # wp = agar_navigation.waypoint(i, wp3[0], wp3[1], wp4[0], wp4[1], True, curr_wp.reverse_turn, curr_wp.tank, 
                #     curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1, 
                #     curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited)
                # self.waypoints.append(wp)

                # pose = PoseStamped()
                # pose.pose.position.x = wp4[0]
                # pose.pose.position.y = wp4[1]
                # self.path.poses.append(pose)

                # start_x = wp4[0]
                # start_y = wp4[1]

            i += 1

        self.get_logger().info(f'Total number of waypoints: {len(self.waypoints)}, total distance: {round(total_distance, 2)}')

        if (len(self.waypoints) > 0):
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
        if msg.data is True and len(self.waypoints) > 0:
            self.navigation_active  = True
            self.current_action     = robot_action.FOLLOW_PATH
        else:
            self.navigation_active  = False
            self.current_action     = robot_action.STOP

    def load_waypoints_callback(self, msg):
        self.reset_waypoints()
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
        if self.navigation_active:
            start_point     = (self.current_waypoint.start_x, self.current_waypoint.start_y)
            end_point       = (self.current_waypoint.end_x, self.current_waypoint.end_y)
            # angle_deg       = self.calculate_angle(end_point)
            distance_in_m   = sqrt(pow(self.robot_position.x - end_point[0], 2) + pow(self.robot_position.y - end_point[1], 2))

            if distance_in_m < self.goal_distance:
                self.goal_reached()
            elif self.current_action == robot_action.FOLLOW_PATH:
                self.follow_path(start_point, end_point, distance_in_m)
            elif self.current_action == robot_action.BACKUP:
                self.backup()

    def goal_reached(self):
        self.waypoint_index += 1
        self.goal_distance = self.default_goal_distance
        if self.waypoint_index == len(self.waypoints) - 1:
            self.goal_distance = 0.3

        if self.waypoint_index < len(self.waypoints):
            self.get_logger().info(f'Moving to waypoint {self.waypoint_index}')
            self.current_waypoint = self.waypoints[self.waypoint_index]
            reverse = False
            self.reverse = False
            if self.waypoint_index < len(self.waypoints) - 1 and self.waypoints[self.waypoint_index + 1].reverse: reverse = True
            if self.current_waypoint.reverse: 
                reverse = True
                # self.current_action = robot_action.REVERSE
                self.reverse = True
            if reverse: self.goal_distance = 0.3
        else:
            self.get_logger().info('All waypoints visited')
            self.current_action     = robot_action.STOP
            self.navigation_active  = False
            
    def follow_path(self, start_point, end_point, distance_in_m):
        try:
            if self.reverse:
                self.robot_position.z = self.robot_position.z + pi

            (point_short, point_long)   = self.point_projection(start_point, end_point, self.path_short_point, self.path_long_point)
            angle_short_deg             = self.calculate_angle(point_short)
            angle_long_deg              = self.calculate_angle(point_long)

            if abs(angle_long_deg) < 30: #15:
                angle_deg = angle_short_deg
                self.publish_pose_path(point_short)
            else:
                angle_deg = angle_long_deg
                self.publish_pose_path(point_long)

            # self.get_logger().info(f'Robot ({round(self.robot_position.x, 2)}, {round(self.robot_position.y, 2)}, {round(self.robot_position.z, 2)}, {round(degrees(self.robot_position.z), 2)})')            
            # self.get_logger().info(f'angle: {round(angle_deg, 2)}, distance: {round(distance_in_m, 2)}, waypoint: {round(self.waypoint_index, 2)}, ({round(self.current_waypoint.end_x, 2)}, {round(self.current_waypoint.end_y, 2)})')

            if abs(angle_deg) > 30: speed_z = 1.0
            elif abs(angle_deg) > 10 and abs(angle_deg) <= 30: speed_z = 0.5
            elif abs(angle_deg) > 2 and abs(angle_deg) <= 10: speed_z = 0.25
            elif abs(angle_deg) > 1 and abs(angle_deg) <= 2: speed_z = 0.1
            else: speed_z = 0.0

            if self.is_slow:
                speed_x = 0.3
            else:
                speed_x = 1.0

            if speed_z > 1: speed_z = 1.0
            if angle_deg < 0: speed_z *= -1.0

            if self.reverse:
                speed_x *= -1

            if speed_x >= 0 and self.robot_direction == robot_direction.BACKWARD:
                self.current_action = robot_action.STOP
                self.robot_direction = robot_direction.FORWARD
                self.timer_stop.reset()
            elif speed_x < 0 and self.robot_direction == robot_direction.FORWARD:
                self.current_action = robot_action.STOP
                self.robot_direction = robot_direction.BACKWARD
                self.timer_stop.reset()

            msg = Twist()
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
            # angle = atan2(trans.transform.translation.y, trans.transform.translation.x)
            # angle_deg = degrees(angle)
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
    
    def stop_and_wait(self):
        self.timer_stop.cancel()
        self.get_logger().info(f'Changing direction to {self.robot_direction}')
        self.current_action = robot_action.FOLLOW_PATH
    #endregion

    #region Angle and point calculation
    def calculate_angle(self, end_point):
        x_r = self.robot_position.x
        y_r = self.robot_position.y
        z_r = self.robot_position.z
        (x_w, y_w) = end_point
        
        delta_x = x_w - x_r
        delta_y = y_w - y_r

        theta = atan2(delta_y, delta_x)
        phi = theta - z_r

        calc_phi = phi
        if abs(phi) > pi:
            if phi < 0:
                calc_phi = (2 * pi) + phi
            else:
                calc_phi = phi - (2 * pi)
        
        return degrees(calc_phi)

    def point_projection(self, wp_start, wp_end, dis_short, dis_long):
        vector_AB = np.array([self.robot_position.x - wp_end[0], self.robot_position.y - wp_end[1]])
        vector_CD = np.array([wp_end[0] - wp_start[0], wp_end[1] - wp_start[1]])
        vector_CD_norm = np.sqrt(sum(vector_CD**2))
        move = (np.dot(vector_AB, vector_CD) / vector_CD_norm**2) * vector_CD

        P = (wp_end[0] + move[0], wp_end[1] + move[1])

        length_PD = sqrt(vector_CD[0]**2 + vector_CD[1]**2)
        unit_vector = (vector_CD[0] / length_PD, vector_CD[1] / length_PD)
        point_short = (P[0] + dis_short * unit_vector[0], P[1] + dis_short * unit_vector[1])
        point_long  = (P[0] + dis_long * unit_vector[0], P[1] + dis_long * unit_vector[1])

        return (point_short, point_long)
    
    def move_point(self, A, B, distance):
        (xa, ya) = A
        (xb, yb) = B
        dAB = sqrt(pow(xb - xa, 2) +  pow(yb - ya, 2))
        dBC = (dAB + distance) / dAB
        
        xc = (1 - dBC) * xa + dBC * xb
        yc = (1 - dBC) * ya + dBC * yb

        return [xc, yc]
    #endregion

    #region Transforms
    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, Time())
            
            self.robot_position.x = trans.transform.translation.x
            self.robot_position.y = trans.transform.translation.y

            orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            (roll, pitch, self.robot_position.z) = euler_from_quaternion(orientation_list)
            self.robot_position.deg = degrees(self.robot_position.z)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
    #endregion

#region Main
def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()
    waypoint_follower.get_logger().info('Closed')

if __name__ == '__main__':
    main()
#endregion