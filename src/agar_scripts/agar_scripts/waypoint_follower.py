#!/usr/bin/env python3
import os
import csv

import future
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from agar_msgs.srv import RobotSpeedSave
from agar_msgs.msg import CollisionZones
from enum import Enum
from std_msgs.msg import String, Bool, Int16
from std_srvs.srv import Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import euler_from_quaternion

from math import atan2, degrees, pi, sqrt
from datetime import datetime
import numpy as np

from std_msgs.msg import Int32MultiArray, UInt64
from rclpy.time import Time

from agar_common.agar_common_library import agar_navigation

class robot_action(Enum):
    FOLLOW_PATH         = 1
    STOP                = 2
    WAIT_TO_STOP        = 3
    SNAKE               = 4
    REVERSE             = 5
    SPIN                = 6
    BACKUP              = 7
    BACKUP_FORWARD      = 8
    NO_ACTION           = 9

class robot_direction(Enum):
    FORWARD         = 1
    BACKWARD        = 2

class RobotPosition(object):
    x       = 0.0
    y       = 0.0
    z       = 0.0
    deg     = 0.0

# ==================== FENCE HELPER FUNKCIJE ====================
def normalize_mission_filename(name: str) -> str:
    """Normalizuje ime misije - dodaje .csv ako ne postoji"""
    name = name.strip()
    if not name:
        return ""
    if not name.lower().endswith(".csv"):
        name = name + ".csv"
    return name


def fence_filename_from_mission(mission_csv: str) -> str:
    """Generiše ime fence fajla iz imena misije"""
    base = os.path.splitext(mission_csv)[0]
    return f"{base}_fence.csv"


def point_in_polygon(x: float, y: float, poly):
    """
    Ray casting algoritam za proveru da li je tačka unutar poligona.
    poly: lista (x,y) temena poligona (ne mora biti zatvoren).
    """
    if len(poly) < 3:
        return False

    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]

        denom = (yj - yi) if (yj - yi) != 0 else 1e-12
        intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / denom + xi)
        if intersect:
            inside = not inside
        j = i
    return inside
# ================================================================


class WaypointFollower(Node):
    #region Initialization
    def __init__(self):
        super().__init__('waypoint_follower')

        self.simulator_mode = True

        self.was_inside_fence = False
        self.last_fence_log_time = 0.0

        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info('Initialization completed')

        timer_period        = 1 / 20  # seconds
        self.timer          = self.create_timer(timer_period, self.run)

        self.timer_stop     = self.create_timer(0.3, self.stop_and_wait)
        self.timer_stop.cancel()

        self.timer_stop_spin = self.create_timer(2.0, self.stop_and_wait_for_spin)
        self.timer_stop_spin.cancel()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_topic          = '/cmd_vel' if self.simulator_mode else '/cmd_vel_raw'
        self.cmd_vel_pub            = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.get_logger().info(f'Velocity command topic: {self.cmd_vel_topic}')

        self.waypoints_path_pub             = self.create_publisher(Path, '/plan', 10)
        self.angle_path_pub                 = self.create_publisher(Path, '/angle_path', 10)
        self.brake_robot_pub                = self.create_publisher(Int16, '/brake_status', 10)

        self.notifications_pub              = self.create_publisher(String, '/agar_notifications', 10)
        self.mission_loaded_pub             = self.create_publisher(Bool, "/mission_loaded", 10)
        self.actuators_position_pub         = self.create_publisher(Int32MultiArray, "/activate_actuators_position", 10)
        self.actuators_position_brake_pub   = self.create_publisher(Int32MultiArray, "/activate_actuators_brake_position", 10)
        self.extra_motor_pub                = self.create_publisher(UInt64, "/extra_motor_status", 10)
        self.visited_waypoint_pub           = self.create_publisher(Int16, "/visited_waypoint", 10)
        self.navigation_active_pub          = self.create_publisher(Bool, '/navigation_active', 10)

        # ========== FENCE PUBLISHER ==========
        self.fence_path_pub                 = self.create_publisher(Path, '/virtual_fence_path', 10)
        # =====================================

        self.navigation_active_sub          = self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)
        self.load_waypoints_sub             = self.create_subscription(String, '/load_waypoints', self.load_waypoints_callback, 10, callback_group=self.callback_group)
        self.reset_waypoints_sub            = self.create_subscription(UInt64, '/reset_waypoints', self.reset_waypoints_callback, 10)
        self.collision_zones_sub            = self.create_subscription(CollisionZones, '/collision_zones', self.collision_zones_callback, 10)

        self.robot_stopped_sub            = self.create_subscription(Bool, '/robot_stopped', self.robot_stopped_callback, 10)


        self.source_frame = 'odom'
        self.target_frame = 'base_footprint'

        self.robot_position = RobotPosition()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.navigation_active          = False
        self.current_action             = robot_action.NO_ACTION
        self.wait_action                = robot_action.NO_ACTION
        self.backup_distance            = 0.0
        self.backup_speed               = -0.3
        self.backup_start_position      = [0, 0]
        self.is_slow                    = False
        self.default_goal_distance      = 2.6
        self.goal_distance              = self.default_goal_distance
        self.reverse                    = False
        self.robot_direction            = robot_direction.FORWARD
        self.reverse_turn_distance      = 1.8
        self.path_short_point           = 1.5
        self.path_long_point            = 8.0

        self.waypoint_index             = 0
        self.current_waypoint           = None
        self.waypoints                  = []
        self.addition_waypoints         = []
        self.waypoints_csv              = None
        self.waypoint_file_name         = None
        primary_waypoints_path          = '/home/agar/ros_data/waypoints' if self.simulator_mode else '/home/nvidia/ros_data/waypoints'
        fallback_waypoints_path         = '/home/nvidia/ros_data/waypoints' if self.simulator_mode else '/home/agar/ros_data/waypoints'
        self.waypoints_path_candidates  = [primary_waypoints_path, fallback_waypoints_path]
        self.waypoints_path             = primary_waypoints_path

        self.path = Path()
        self.path.header.frame_id = 'map'

        # ========== FENCE STANJE ==========
        self.fence_ll = []          # [(lat, lon), ...]
        self.fence_map = []         # [(x, y), ...] u map koordinatama
        self.fence_margin = 0.3     # margin u metrima od ivice ograde (bezbednosna zona)
        self.map_frame = 'map'
        self.fence_violation_logged = False
        # ==================================

        if not self.simulator_mode:
            self.cli_speed = self.create_client(RobotSpeedSave, '/robot_speed_save_server')
            while not self.cli_speed.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service robot_speed_save_server not available, waiting again...')
            self.req_speed = RobotSpeedSave.Request()

        self.cli = self.create_client(FromLL, '/fromLL', callback_group=self.callback_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service fromLL not available, waiting again...')
        self.req = FromLL.Request()

        # ========== FENCE FROMLL REQUEST ==========
        self.req_fromll = FromLL.Request()
        # ==========================================

    #endregion


    # ==================== FENCE FUNKCIJE ====================
    def load_fence_file(self, fence_path: str):
        """Učitava fence tačke iz CSV fajla"""
        pts = []
        with open(fence_path, "r", newline="") as f:
            reader = csv.reader(f, delimiter="|")
            for row in reader:
                if not row:
                    continue
                try:
                    lat = float(row[0])
                    lon = float(row[1])
                    pts.append((lat, lon))
                except Exception:
                    continue

        # Izbaci duplikate koji su isti jedan za drugim
        cleaned = []
        for p in pts:
            if not cleaned or (abs(cleaned[-1][0] - p[0]) > 1e-12 or abs(cleaned[-1][1] - p[1]) > 1e-12):
                cleaned.append(p)

        self.fence_ll = cleaned

    def ll_to_map_xy(self, lat, lon, alt=0.0):
        """Pretvara lat/lon u map x/y koordinate preko /fromLL servisa"""
        p = GeoPoint()
        p.latitude = float(lat)
        p.longitude = float(lon)
        p.altitude = float(alt)
        self.req_fromll.ll_point = p
        future = self.cli.call_async(self.req_fromll)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return None
        return (res.map_point.x, res.map_point.y)

    def build_map_polygon_from_ll(self) -> bool:
        """Pretvara sve fence lat/lon tačke u map koordinate"""
        if len(self.fence_ll) < 3:
            self.fence_map = []
            return False

        poly = []
        for (lat, lon) in self.fence_ll:
            xy = self.ll_to_map_xy(lat, lon, 0.0)
            if xy is None:
                return False
            poly.append(xy)

        self.fence_map = poly
        return True

    def publish_fence_path(self):
        """Objavljuje fence kao Path za vizualizaciju u RViz"""
        if not self.fence_map:
            return

        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.fence_map:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            msg.poses.append(ps)

        # Zatvori vizuelno (dodaj prvu tačku na kraj)
        if self.fence_map:
            x0, y0 = self.fence_map[0]
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x0)
            ps.pose.position.y = float(y0)
            ps.pose.position.z = 0.0
            msg.poses.append(ps)

        self.fence_path_pub.publish(msg)

    def is_inside_fence(self, x, y):
        """Proverava da li je tačka (x,y) unutar fence ograde"""
        if not self.fence_map:
            return True  # Ako nema fence-a, dozvoli kretanje
        return point_in_polygon(x, y, self.fence_map)

    def distance_to_fence_edge(self, x, y):
        """
        Računa najmanju udaljenost od tačke (x,y) do najbliže ivice fence-a.
        Koristi se za preventivno zaustavljanje na margini.
        """
        if not self.fence_map:
            return float('inf')

        min_dist = float('inf')
        n = len(self.fence_map)

        for i in range(n):
            x1, y1 = self.fence_map[i]
            x2, y2 = self.fence_map[(i + 1) % n]

            # Rastojanje od tačke do linije (segment fence-a)
            dist = self._point_to_segment_distance(x, y, x1, y1, x2, y2)
            min_dist = min(min_dist, dist)

        return min_dist

    def _point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Računa najkraće rastojanje od tačke (px,py) do segmenta linije (x1,y1)-(x2,y2)"""
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return sqrt((px - x1)**2 + (py - y1)**2)

        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)))
        projection_x = x1 + t * dx
        projection_y = y1 + t * dy

        return sqrt((px - projection_x)**2 + (py - projection_y)**2)

    def check_virtual_fence_boundary(self):
        """Proverava da li je robot na ivici ograde ili van nje."""
        if not self.fence_map:
            return True

        rx = self.robot_position.x
        ry = self.robot_position.y

        inside = self.is_inside_fence(rx, ry)

        # ========== DOZVOLI ULAZAK U OGRADU ==========
        if inside:
            self.was_inside_fence = True  # Robot je ušao u ogradu
            self.fence_violation_logged = False
            return True

        # ========== BLOKIRAJ SAMO AKO JE BIO UNUTRA PA IZAŠAO ==========
        if not inside and self.was_inside_fence:
            # Robot je bio unutra i sada je IZAŠAO - VIOLATION!
            if not self.fence_violation_logged:
                self.get_logger().error("=" * 80)
                self.get_logger().error("FENCE VIOLATION: Robot EXITED the virtual fence!")
                self.get_logger().error("=" * 80)

                self.get_logger().warn(f">>> Sending BRAKE command (0, 0) to {self.cmd_vel_topic}")
                self.brake_robot()

                self.get_logger().warn(">>> Setting navigation_active = False (LOCAL ONLY)")
                self.navigation_active = False  # <-- SAMO LOKALNA PROMENLJIVA
                # NE ŠALJI NA TOPIC! To bi pozvalo callback ponovo!

                self.get_logger().warn(">>> Sending notification to /agar_notifications")
                self.send_notifications("FENCE VIOLATION: Robot exited virtual fence area!")

                self.fence_violation_logged = True
                self.get_logger().error("=" * 80)
            return False

        # Robot je VAN ograde ali NIJE bio unutra - dozvoli da uđe
        if not inside and not self.was_inside_fence:
            self.get_logger().info(f"Robot outside fence but approaching entry. Allowing navigation.")
            return True

        # Proveri margin samo ako je unutra
        dist_to_edge = self.distance_to_fence_edge(rx, ry)
        if dist_to_edge < self.fence_margin:
            if not self.fence_violation_logged:
                self.get_logger().warn("=" * 80)
                self.get_logger().warn(f"FENCE WARNING: Robot too close to fence edge ({dist_to_edge:.2f}m < {self.fence_margin}m)")
                self.get_logger().warn("=" * 80)

                self.get_logger().warn(f">>> Sending BRAKE command (0, 0) to {self.cmd_vel_topic}")
                self.brake_robot()

                self.get_logger().warn(">>> Setting navigation_active = False (LOCAL ONLY)")
                self.navigation_active = False  # <-- SAMO LOKALNA PROMENLJIVA
                # NE ŠALJI NA TOPIC!

                self.get_logger().warn(">>> Sending notification to /agar_notifications")
                self.send_notifications(f"Robot too close to fence edge ({dist_to_edge:.2f}m). Navigation stopped.")

                self.fence_violation_logged = True
                self.get_logger().warn("=" * 80)
            return False

        self.fence_violation_logged = False
        return True




    # ========================================================

    #region Load waypoints
    def mission_file_path(self):
        return os.path.join(self.waypoints_path, self.waypoint_file_name)

    def resolve_waypoints_path(self, mission_name):
        for base_path in self.waypoints_path_candidates:
            if os.path.exists(os.path.join(base_path, mission_name)):
                return base_path
        return self.waypoints_path_candidates[0]

    def reset_waypoints(self, csv_reset):
        self.current_action         = robot_action.NO_ACTION
        self.robot_direction        = robot_direction.FORWARD
        self.navigation_active      = False
        self.goal_distance          = self.default_goal_distance
        self.path                   = Path()
        self.path.header.frame_id   = 'map'

        self.waypoint_index         = 0
        self.current_waypoint       = None
        self.waypoints              = []

        if csv_reset:
            self.waypoints_csv          = None
            self.waypoint_file_name     = None

    def load_waypoints_from_file(self):
        mission_path = self.mission_file_path()
        self.get_logger().info(f"Loading waypoints from: {mission_path}")
        try:
            with open(mission_path, 'r') as file:
                reader = csv.DictReader(file, delimiter = '|', fieldnames=agar_navigation.csv_header)
                data = list(reader)
                self.waypoints_csv = [agar_navigation.waypoint_csv(i) for i in data]
                self.generate_waypoint_list()
        except Exception as ex:
            self.get_logger().error(f"ERROR loading waypoints: {str(ex)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def generate_waypoint_list(self):
        self.robot_direction        = robot_direction.FORWARD

        number_of_visited = 0

        pose = PoseStamped()
        pose.pose.position.x = self.robot_position.x
        pose.pose.position.y = self.robot_position.y
        self.path.poses.append(pose)

        is_first = False

        i = 0
        start_x = self.robot_position.x
        start_y = self.robot_position.y
        total_distance = 0
        number_of_visited = -1
        while i < len(self.waypoints_csv):
            curr_wp = self.waypoints_csv[i]
            visited = curr_wp.visited

            if not visited:
                (pos_x, pos_y) = self.convertLLToXY(curr_wp.latitude, curr_wp.longitude)
                self.get_logger().info(f'Position x: {pos_x}, y: {pos_y}')

                pose = PoseStamped()
                pose.pose.position.x = pos_x
                pose.pose.position.y = pos_y
                self.path.poses.append(pose)

                wp = agar_navigation.waypoint(i, start_x, start_y, pos_x, pos_y, curr_wp.reverse, curr_wp.reverse_turn, curr_wp.tank,
                    curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1,
                    curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited, False)
                self.waypoints.append(wp)

                total_distance += sqrt(pow(start_x - pos_x, 2) + pow(start_y - pos_y, 2))
                if not is_first:
                    self.current_waypoint = wp
                    is_first = True

                start_x = pos_x
                start_y = pos_y

                if curr_wp.reverse_turn and i < len(self.waypoints_csv) - 2:
                    a_end_x = wp.end_x
                    a_end_y = wp.end_y
                    (b_start_x, b_start_y) = self.convertLLToXY(self.waypoints_csv[i + 1].latitude, self.waypoints_csv[i + 1].longitude)

                    wp1 = self.move_point((a_end_x, a_end_y), (b_start_x, b_start_y), self.reverse_turn_distance)
                    wp2 = self.move_point((b_start_x, b_start_y), (a_end_x, a_end_y), self.reverse_turn_distance)

                    pose = PoseStamped()
                    pose.pose.position.x = wp1[0]
                    pose.pose.position.y = wp1[1]
                    self.path.poses.append(pose)

                    wp = agar_navigation.waypoint(i + 1, a_end_x, a_end_y, wp1[0], wp1[1], False, curr_wp.reverse_turn, curr_wp.tank,
                        curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1,
                        curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited, True)
                    self.waypoints.append(wp)

                    total_distance += sqrt(pow(wp1[0] - a_end_x, 2) + pow(wp1[1] - a_end_y, 2))

                    pose = PoseStamped()
                    pose.pose.position.x = wp2[0]
                    pose.pose.position.y = wp2[1]
                    self.path.poses.append(pose)

                    wp = agar_navigation.waypoint(i + 1, b_start_x, b_start_y, wp2[0], wp2[1], True, curr_wp.reverse_turn, curr_wp.tank,
                        curr_wp.speed, curr_wp.agar_action, curr_wp.combo_value, curr_wp.combo_text, curr_wp.param_1,
                        curr_wp.param_2, curr_wp.param_3, curr_wp.param_4, curr_wp.visited, True)
                    self.waypoints.append(wp)

                    total_distance += sqrt(pow(wp1[0] - b_start_x, 2) + pow(wp1[1] - b_start_y, 2))
            else:
                number_of_visited += 1
            i += 1

        self.get_logger().info(f'Total number of waypoints: {len(self.waypoints)}, total distance: {round(total_distance, 2)}')

        if len(self.waypoints) > 0:
            if not self.simulator_mode:
                self.set_robot_speed()

        if len(self.waypoints) > 2 and self.waypoints[1].tank:
            self.goal_distance = 0.7

        msg = Int16()
        msg.data = number_of_visited
        self.visited_waypoint_pub.publish(msg)

        if (len(self.waypoints) > 0):
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.waypoints_path_pub.publish(self.path)

        msgMisLoaded = Bool()
        msgMisLoaded.data = True
        self.mission_loaded_pub.publish(msgMisLoaded)

        noty = 'Mission {0} loaded. '.format(self.waypoint_file_name)
        noty += 'Number of waypoints: {0}, number of visited: {1}, total distance: {2}m.'.format(len(self.waypoints), number_of_visited, round(total_distance, 2))
        self.send_notifications(noty)

    def update_waypoints_to_file(self, wp_idx):
        visited = self.waypoints_csv[wp_idx].visited
        if visited != True:
            self.get_logger().info(f'Waypoint {wp_idx} Updated to visited')
            self.waypoints_csv[wp_idx].visited = True

            msg = Int16()
            msg.data = wp_idx
            self.visited_waypoint_pub.publish(msg)

            try:
                with open(self.mission_file_path(), 'w') as f:
                    for x in self.waypoints_csv:
                        f.write(f'{x.latitude}|{x.longitude}|{x.reverse}|{x.reverse_turn}|{x.tank}|{x.speed}|{x.agar_action}|{x.combo_value}|{x.combo_text}|{x.param_1}|{x.param_2}|{x.param_3}|{x.param_4}|{x.visited}\n')
            except Exception as ex:
                self.get_logger().error(f"Error: {ex}")
    #endregion

    #region Subscriber callback
    def collision_zones_callback(self, msg):

        if self.is_slow == False and msg.front_slow == True:
            self.get_logger().warn("Collision ahead. Entering slow speed mode.")
        elif self.is_slow == True and msg.front_slow == False:
            self.get_logger().warn("Exiting slow speed mode.")

        self.is_slow = msg.front_slow

    def navigation_active_callback(self, msg):

        # self.get_logger().info('********************************************************************')
        # self.get_logger().info('Navigation active callback received: {0}'.format(msg.data))
        # self.get_logger().info('Number of waypoints: {0}'.format(len(self.waypoints)))
        # self.get_logger().info('********************************************************************')

        if msg.data is True:
            # Proveri da li ima waypoints
            if len(self.waypoints) == 0:
                self.get_logger().warn("Cannot start navigation: No waypoints loaded!")
                return

            # Proveri da li je waypoint_index validan
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().warn(f"Cannot start navigation: waypoint_index ({self.waypoint_index}) >= len(waypoints) ({len(self.waypoints)})")
                return

            # Sve je OK - pokreni navigaciju
            self.navigation_active = True
            self.current_action = robot_action.FOLLOW_PATH
            self.send_notifications(f"Navigation started. Start time: {datetime.now()}")
            self.get_logger().info("*" * 70)
            self.get_logger().info("Mission started.")
            self.get_logger().info(f"Start time: {datetime.now()}")
            self.get_logger().info("*" * 70)
        else:
            # Eksplicitno stopiranje navigacije
            self.get_logger().info("*" * 70)
            self.get_logger().info("Navigation stopped by user.")
            self.get_logger().info("*" * 70)
            self.navigation_active = False
            self.current_action = robot_action.STOP
            self.brake_robot()

    def load_waypoints_callback(self, msg):
        """
        Učitava waypoint fajl i automatski traži odgovarajući fence fajl
        """
        self.reset_waypoints(True)

        # ========== NORMALIZUJ IME MISIJE ==========
        mission = normalize_mission_filename(msg.data)
        self.waypoint_file_name = mission
        self.waypoints_path = self.resolve_waypoints_path(self.waypoint_file_name)
        mission_path = self.mission_file_path()
        # ===========================================

        self.get_logger().info(f"Attempting to load mission: {mission_path}")
        self.get_logger().info(f"Mission search paths: {self.waypoints_path_candidates}")

        # Učitaj misiju
        if os.path.exists(mission_path):
            self.load_waypoints_from_file()
        else:
            self.get_logger().warn(f'The mission file does not exist: {mission_path}')
            return

        # ========== POKUŠAJ DA UČITAŠ FENCE FAJL ==========
        fence_csv = fence_filename_from_mission(self.waypoint_file_name)
        fence_path = os.path.join(self.waypoints_path, fence_csv)

        if not os.path.exists(fence_path):
            # Ako nema fence fajla, očisti fence i nastavi bez ograde
            self.fence_ll = []
            self.fence_map = []
            self.get_logger().warn(f"Fence file NOT found: {fence_path}")
            self.send_notifications(f"Fence file not found: {fence_csv}")
            return

        self.load_fence_file(fence_path)
        ok = self.build_map_polygon_from_ll()
        if not ok:
            self.fence_ll = []
            self.fence_map = []
            self.get_logger().warn("Fence load failed: /fromLL conversion failed.")
            self.send_notifications("Fence load failed: /fromLL conversion failed.")
            return

        self.get_logger().info(f"Loaded fence: {fence_csv} points={len(self.fence_ll)}")
        if self.fence_map:
            x0, y0 = self.fence_map[0]
            self.get_logger().info(f"Fence map origin sample: x={x0:.3f}, y={y0:.3f}")
        self.send_notifications(f"Fence loaded: {fence_csv}, points={len(self.fence_ll)}, margin={self.fence_margin}m")
        # ==================================================

    def set_datum_callback(self, request, response):
        response.success = False
        try:
            self.datum_latitude     = request.datum.position.latitude
            self.datum_longitude    = request.datum.position.longitude
            self.get_logger().info(f'Datum received: {self.datum_latitude}, {self.datum_longitude}')
        except Exception as ex:
            self.get_logger().error(f'Exception: {ex}')

        response.success = True
        return response

    def reset_waypoints_callback(self, msg):
        if self.waypoints_csv is None:
            return

        rec_no = msg.data
        self.send_notifications(f'Reset waypoints to {rec_no}, total number of waypoints: {len(self.waypoints_csv)}')
        self.get_logger().info(f'Reset waypoints to {rec_no}, total number of waypoints: {len(self.waypoints_csv)}')
        rec_no -= 1

        for idx, wp in enumerate(self.waypoints_csv):
            if idx >= rec_no: wp.visited = False
            else: wp.visited = True

        try:
            with open(self.mission_file_path(), 'w') as f:
                for x in self.waypoints_csv:
                    f.write(f'{x.latitude}|{x.longitude}|{x.reverse}|{x.reverse_turn}|{x.tank}|{x.speed}|{x.agar_action}|{x.combo_value}|{x.combo_text}|{x.param_1}|{x.param_2}|{x.param_3}|{x.param_4}|{x.visited}\n')

            self.send_notifications('Waypoints reseted.')
            self.get_logger().info('Waypoints reseted.')
            self.all_waypoint_reached = False
            self.reset_waypoints(False)
            self.generate_waypoint_list()
        except Exception as ex:
            self.get_logger().error(f"Error: {ex}")
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

    def brake_robot(self):
        self.get_logger().warn(f"    → brake_robot() called - Publishing Twist(linear.x=0.0, angular.z=0.0) to {self.cmd_vel_topic}")
        cmdvel = Twist()
        cmdvel.linear.x = 0.0
        cmdvel.linear.y = 0.0
        self.cmd_vel_pub.publish(cmdvel)
        self.get_logger().warn(f"    → {self.cmd_vel_topic} message SENT")
    #endregion

    #region Path following
    def run(self):
        self.get_robot_position()

        # ========== PUBLISH FENCE PATH ZA RVIZ ==========
        self.publish_fence_path()
        # ================================================

        # ========== DEBUG LOG ==========
        if len(self.fence_map) > 0:
            current_time = self.get_clock().now().nanoseconds / 1e9  # Vreme u sekundama
            if current_time - self.last_fence_log_time >= 2.0:  # Svakih 2 sekunde
                self.last_fence_log_time = current_time
                rx = self.robot_position.x
                ry = self.robot_position.y
                inside = self.is_inside_fence(rx, ry)
                dist = self.distance_to_fence_edge(rx, ry)
                self.get_logger().warn(f"FENCE CHECK: Robot ({rx:.2f}, {ry:.2f}) | Inside={inside} | Dist={dist:.2f}m")
        # ================================

        # if len(self.fence_map) > 0:
        #     if not self.check_virtual_fence_boundary():
        #         return

        # ========== PROVERI VIRTUAL FENCE - ODMAH ZAUSTAVI AKO JE VAN ==========
        if self.navigation_active:
            if not self.check_virtual_fence_boundary():
                # Fence je zaustavila robota - izađi odmah
                return
        # ========================================================================

        # ========== ORIGINALNA LOGIKA - NETAKNUTA ==========
        if self.navigation_active:
            start_point     = (self.current_waypoint.start_x, self.current_waypoint.start_y)
            end_point       = (self.current_waypoint.end_x, self.current_waypoint.end_y)
            distance_in_m   = sqrt(pow(self.robot_position.x - end_point[0], 2) + pow(self.robot_position.y - end_point[1], 2))

            if distance_in_m < self.goal_distance and self.current_action == robot_action.FOLLOW_PATH:
                self.goal_reached()
            elif self.current_action == robot_action.SPIN:
                self.spin(end_point)
            elif self.current_action == robot_action.FOLLOW_PATH:
                self.follow_path(start_point, end_point, distance_in_m)
            elif self.current_action == robot_action.BACKUP:
                self.backup()
            elif self.current_action == robot_action.BACKUP_FORWARD:
                self.backup_forward()
        # ====================================================

    def goal_reached(self):
        if self.waypoints[self.waypoint_index].generated_wp == False:
            self.update_waypoints_to_file(self.waypoints[self.waypoint_index].idx)

        agar_action = self.waypoints[self.waypoint_index].agar_action
        if agar_action > 0:
            param_1 = self.waypoints[self.waypoint_index].param_1
            param_2 = self.waypoints[self.waypoint_index].param_2
            param_3 = self.waypoints[self.waypoint_index].param_3
            param_4 = self.waypoints[self.waypoint_index].param_4
            self.get_logger().info(f'Performing agar action {agar_action} with parameters {param_1}, {param_2}, {param_3}, {param_4}')
            self.perform_agar_action(agar_action, param_1, param_2, param_3, param_4)

        self.waypoint_index += 1
        self.goal_distance = self.default_goal_distance
        if self.waypoint_index == len(self.waypoints) - 1:
            self.goal_distance = 0.3

        if self.waypoint_index < len(self.waypoints):
            self.get_logger().info(f'Moving to waypoint {self.waypoint_index}')

            self.current_waypoint = self.waypoints[self.waypoint_index]
            reverse = False
            self.reverse = False

            if not self.simulator_mode:
                self.set_robot_speed()
                self.set_robot_speed()

            if self.waypoint_index < len(self.waypoints) - 1 and self.waypoints[self.waypoint_index + 1].tank:
                self.goal_distance = 0.7

            if self.current_waypoint.tank:
                self.brake_robot()
                self.current_action = robot_action.STOP
                self.timer_stop_spin.reset()
            else:
                if self.waypoint_index < len(self.waypoints) - 1 and self.waypoints[self.waypoint_index + 1].reverse: reverse = True
                if self.current_waypoint.reverse:
                    reverse = True
                    self.reverse = True
                if reverse: self.goal_distance = 0.7
        else:
            self.get_logger().info('********************************************************************')
            self.get_logger().info('All waypoint reached')
            self.get_logger().info('End time: {0}'.format(datetime.now()))
            self.get_logger().info('********************************************************************')
            self.current_action     = robot_action.STOP
            self.navigation_active  = False
            msg_nav_active = Bool()
            msg_nav_active.data = False
            self.navigation_active_pub.publish(msg_nav_active)
            self.brake_robot()
            self.send_notifications('All waypoint reached. End time: {0}'.format(datetime.now()))

    def follow_path(self, start_point, end_point, distance_in_m):
        try:
            if self.reverse:
                self.robot_position.z = self.robot_position.z + pi

            (point_short, point_long)   = self.point_projection(start_point, end_point, self.path_short_point, self.path_long_point)
            angle_short_deg             = self.calculate_angle(point_short)
            angle_long_deg              = self.calculate_angle(point_long)

            if abs(angle_long_deg) < 30:
                angle_deg = angle_short_deg
                self.publish_pose_path(point_short)
            else:
                angle_deg = angle_long_deg
                self.publish_pose_path(point_long)

            if self.simulator_mode:
                if abs(angle_deg) > 30: speed_z = 1.0
                elif abs(angle_deg) > 10 and abs(angle_deg) <= 30: speed_z = 0.5
                elif abs(angle_deg) > 2 and abs(angle_deg) <= 10: speed_z = 0.25
                elif abs(angle_deg) > 1 and abs(angle_deg) <= 2: speed_z = 0.1
                else: speed_z = 0.0
            else:
                if abs(angle_deg) > 10: speed_z = 1.0
                elif abs(angle_deg) > 8 and abs(angle_deg) <= 10: speed_z = 0.5
                elif abs(angle_deg) > 3 and abs(angle_deg) <= 8: speed_z = 0.1
                else: speed_z = 0.0

            if self.is_slow:
                speed_x = 0.3
            else:
                speed_x = 1.0

            if speed_z > 1: speed_z = 1.0
            if angle_deg < 0: speed_z *= -1.0

            if self.reverse:
                speed_x *= -1
                if not self.simulator_mode:
                    speed_z *= -1

            if speed_x >= 0 and self.robot_direction == robot_direction.BACKWARD:
                speed_x = 0.0
                speed_z = 0.0
                self.current_action     = robot_action.STOP
                self.robot_direction    = robot_direction.FORWARD
                self.wait_action        = robot_action.FOLLOW_PATH
            elif speed_x < 0 and self.robot_direction == robot_direction.FORWARD:
                speed_x = 0.0
                speed_z = 0.0
                self.current_action     = robot_action.STOP
                self.robot_direction    = robot_direction.BACKWARD
                self.wait_action        = robot_action.FOLLOW_PATH

            msg = Twist()
            msg.linear.x = speed_x
            msg.angular.z = speed_z
            self.cmd_vel_pub.publish(msg)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to waypoint_01: {ex}')

    def spin(self, end_point):
        angle_waypoint = self.calculate_angle(end_point)
        if abs(angle_waypoint) < 10:
            if self.waypoint_index < len(self.waypoints) - 1 and self.waypoints[self.waypoint_index + 1].tank:
                self.goal_distance = 0.5
            else:
                self.goal_distance = self.default_goal_distance
            self.current_action = robot_action.FOLLOW_PATH
        else:
            speed_z = 1.0
            if angle_waypoint < 0: speed_z *= -1.0
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = speed_z
            self.cmd_vel_pub.publish(msg)

    def backup(self):
        try:
            distance_in_m   = sqrt(pow(self.robot_position.x - self.backup_start_position[0], 2) + pow(self.robot_position.y - self.backup_start_position[1], 2))

            if distance_in_m > self.backup_distance:
                self.get_logger().warn("Backup completed. Perform path following.")
                self.backup_distance = 0
                self.current_action = robot_action.FOLLOW_PATH
                self.backup_start_position = [0, 0]

            msg = Twist()
            msg.linear.x = self.backup_speed
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to waypoint_{self.waypoint_index}: {ex}')

    def backup_forward(self):
        try:
            distance_in_m   = sqrt(pow(self.robot_position.x - self.backup_start_position[0], 2) + pow(self.robot_position.y - self.backup_start_position[1], 2))
            if distance_in_m > self.backup_distance:
                self.get_logger().warn("Backup forward completed. Perform path following.")
                self.backup_distance = 0
                self.current_action = robot_action.FOLLOW_PATH
                self.backup_start_position = [0, 0]

            msg = Twist()
            msg.linear.x = self.backup_speed * -1
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to waypoint_{self.waypoint_index}: {ex}')

    def stop_and_wait(self):
        self.get_logger().info(f'Stop and wait finished. Continuing with action {self.wait_action}')
        self.timer_stop.cancel()
        self.current_action     = self.wait_action
        self.wait_action        = robot_action.NO_ACTION

    def robot_stopped_callback(self, msg):
        self.get_logger().info(f'wait action: {self.wait_action}')
        if msg.data and self.navigation_active and self.wait_action != robot_action.NO_ACTION:
            self.get_logger().info('Robot stopped detected. Continuing with next action.')
            self.timer_stop.reset()

    def stop_and_wait_for_spin(self):
        self.timer_stop_spin.cancel()
        self.current_action = robot_action.SPIN

    def perform_agar_action(self, agar_action, param_1, param_2, param_3, param_4):
        if agar_action == 1:
            msg_actuators = Int32MultiArray()
            msg_actuators.data = [param_1, param_2, param_3, param_4]
            self.actuators_position_pub.publish(msg_actuators)
            self.actuators_position_pub.publish(msg_actuators)
            self.actuators_position_pub.publish(msg_actuators)
        elif agar_action == 2:
            msg = UInt64()
            msg.data = 1
            self.extra_motor_pub.publish(msg)
            self.extra_motor_pub.publish(msg)
            self.extra_motor_pub.publish(msg)
        elif agar_action == 3:
            msg = UInt64()
            msg.data = 0
            self.extra_motor_pub.publish(msg)
            self.extra_motor_pub.publish(msg)
            self.extra_motor_pub.publish(msg)
        elif agar_action == 9:
            brake_strength = param_1
            self.get_logger().info(f'Mehanicka kocnica, strength: {brake_strength}')
            msg_actuators_brake = Int32MultiArray()
            msg_actuators_brake.data = [param_1, param_2]
            self.actuators_position_brake_pub.publish(msg_actuators_brake)
            self.actuators_position_brake_pub.publish(msg_actuators_brake)
            self.actuators_position_brake_pub.publish(msg_actuators_brake)
        elif agar_action == 10:
            self.backup_start_position = [self.robot_position.x, self.robot_position.y]
            self.backup_distance = param_1
            self.get_logger().info(f'Backup distance: {round(self.backup_distance, 2)}')

            self.brake_robot()
            self.current_action     = robot_action.STOP
            self.robot_direction    = robot_direction.BACKWARD
            self.wait_action        = robot_action.BACKUP
        elif agar_action == 11:
            self.brake_robot()
            self.backup_start_position = [self.robot_position.x, self.robot_position.y]
            self.backup_distance = param_1
            self.get_logger().info(f'Forward distance: {round(self.backup_distance, 2)}')

            self.current_action     = robot_action.STOP
            self.robot_direction    = robot_direction.FORWARD
            self.wait_action        = robot_action.BACKUP_FORWARD
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
            self.get_logger().warn(f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
    #endregion

    #region Other
    def send_notifications(self, text):
        try:
            msg = String()
            msg.data = text
            self.notifications_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error('Error %r' % exc)

    def set_navigation_active(self, active):
        self.navigation_active = active
        if not self.navigation_active:
            self.current_action     = robot_action.STOP
            self.navigation_active  = False
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            self.brake_robot()
        else:
            self.send_notifications('Navigation started.')
            self.current_action     = robot_action.FOLLOW_PATH
            self.navigation_active  = True

    def release_brake(self):
        msg = Int16()
        msg.data = 0
        self.brake_robot_pub.publish(msg)

    def get_datum(self):
        self.get_logger().info('Getting datum')
        try:
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            res = self.future.result()
            self.datum_latitude     = res.datum.position.latitude
            self.datum_longitude    = res.datum.position.longitude
            self.get_logger().info(f'Datum: {self.datum_latitude}, {self.datum_longitude}')
        except Exception as exc:
            self.get_logger().error('Service call failed %r' % exc)

    def reset_initial_data(self):
        self.waypoints                  = []

    def set_robot_speed(self):
        speed = int(self.waypoints[self.waypoint_index].speed * 100)
        self.get_logger().info(f'Setting robot speed: {speed}')
        try:
            self.req_speed.speed = speed
            self.req_speed.tenk_speed = 50
            future = self.cli_speed.call_async(self.req_speed)
            future.result()
            self.get_logger().info('Service save speed call completed')
        except Exception as exc:
            self.get_logger().error('Service call failed %r' % exc)

    def convertLLToXY(self, latitude, longitude, altitude=0.0):
        try:
            self.get_logger().info('Converting latitude: {0}, longitude: {1}, altitude: {2} to XY'.format(latitude, longitude, altitude))
            point = GeoPoint()
            point.latitude  = latitude
            point.longitude = longitude
            point.altitude  = altitude
            self.req.ll_point = point

            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            response = self.future.result()

            self.get_logger().info('x: {0}, y: {1}, z: {2}'.format(response.map_point.x, response.map_point.y, response.map_point.z))

            return (response.map_point.x, response.map_point.y)

        except Exception as exc:
            self.get_logger().error('Service call failed %r' % exc)

        return (0.0, 0.0)
    #endregion


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
