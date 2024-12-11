import rclpy
from math import degrees, atan2, sqrt
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.node import Node

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from geometry_msgs.msg import TransformStamped, Twist

from random import randrange
from enum import Enum

class robot_action(Enum):
    FOLLOW_PATH     = 1
    STOP            = 2
    SNAKE           = 3
    REVERSE         = 4
    BACKUP          = 5


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
        self.timer = self.create_timer(0.01, self.publish_waypoint)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_drive_base_controller/cmd_vel_unstamped', 10)

        self.source_frame = 'odom'
        self.target_frame = 'base_footprint'

        # self.pos_x = self.pos_y = self.yaw = 0
        self.robot_position = RobotPosition()
        self.waypoint       = RobotPosition()
        self.waypoint.x     = float(randrange(20))
        self.waypoint.y     = float(randrange(20))

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.waypoint_index = 1
        self.publish_waypoint()

        self.current_action = robot_action.FOLLOW_PATH

        self.backup_distance = 0.0

    #endregion

    #region Path Following
    def run(self):
        self.get_robot_position()

        if self.current_action == robot_action.FOLLOW_PATH:
            self.follow_path()
        if self.current_action == robot_action.BACKUP:
            self.backup()
            
        # self.log()

    def follow_path(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', f'waypoint_{self.waypoint_index}', Time())
            distance_in_m = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            angle = atan2(trans.transform.translation.y, trans.transform.translation.x)
            angle_deg = degrees(angle)
            self.get_logger().info(f'Distance: {round(distance_in_m, 2)}, angle waypoint: {round(angle, 2)}, {round(angle_deg, 2)}')

            if abs(angle_deg) > 30: speed_z = 1.0
            elif abs(angle_deg) > 10 and abs(angle_deg) <= 30: speed_z = 0.5
            elif abs(angle_deg) > 2 and abs(angle_deg) <= 10: speed_z = 0.25
            elif abs(angle_deg) > 1 and abs(angle_deg) <= 2: speed_z = 0.1
            else: speed_z = 0.0

            if distance_in_m < 0.1:
                self.backup_distance = distance_in_m + 0.5
                self.current_action = robot_action.BACKUP
            
            speed_x = 1.0
            if speed_z > 1: speed_z = 1.0
            if angle_deg < 0: speed_z *= -1.0

            self.get_logger().info(f'Speed x: {speed_x}, z: {speed_z}')

            msg = Twist()
            # msg.header.stamp = self.get_clock().now().to_msg()
            msg.linear.x = speed_x
            msg.angular.z = speed_z
            self.cmd_vel_pub.publish(msg)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform base_footprint to waypoint_01: {ex}')
        

    def backup(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', f'waypoint_{self.waypoint_index}', Time())
            distance_in_m = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            angle = atan2(trans.transform.translation.y, trans.transform.translation.x)
            angle_deg = degrees(angle)
            self.get_logger().info(f'Distance: {round(distance_in_m, 2)}, angle waypoint: {round(angle, 2)}, {round(angle_deg, 2)}')
            
            if distance_in_m > self.backup_distance:
                self.waypoint_index += 1
                self.waypoint.x = float(randrange(20))
                self.waypoint.y = float(randrange(20))
                self.publish_waypoint()

                self.backup_distance = 0
                self.current_action = robot_action.FOLLOW_PATH

            msg = Twist()
            msg.linear.x = -0.3
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform base_footprint to waypoint_{self.waypoint_index}: {ex}')



    #endregion

    #region transforms
    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, Time())
            
            self.robot_position.x = trans.transform.translation.x
            self.robot_position.y = trans.transform.translation.y

            orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            (roll, pitch, self.robot_position.z) = euler_from_quaternion(orientation_list)
            self.robot_position.deg = degrees(self.robot_position.z)
            
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            
    def publish_waypoint(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'waypoint_{self.waypoint_index}'
        t.transform.translation.x = self.waypoint.x
        t.transform.translation.y = self.waypoint.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)
        self.tf_static_broadcaster.sendTransform(t)
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