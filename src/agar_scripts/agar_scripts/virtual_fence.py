#!/usr/bin/env python3
"""
ROS2 virtual fence guard (standalone node).

Main goals:
- Load `<mission>_fence.csv` when mission is loaded.
- Convert fence LL points to `map` XY using /fromLL.
- Continuously verify robot is inside polygon in the same frame.
- Stop robot + deactivate navigation on fence violation.

This version fixes the common map/odom mismatch by checking robot pose in `map`
(TF map->base_footprint or GPS->/fromLL fallback).
"""

import csv
import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Path
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int16, String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_mission_filename(name: str) -> str:
    mission = name.strip()
    if not mission:
        return ""
    if not mission.lower().endswith(".csv"):
        mission += ".csv"
    return mission


def fence_filename_from_mission(mission_csv: str) -> str:
    return f"{os.path.splitext(mission_csv)[0]}_fence.csv"


def point_in_polygon(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    if len(poly) < 3:
        return False

    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]

        denom = (yj - yi) if (yj - yi) != 0.0 else 1e-12
        intersects = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / denom + xi)
        if intersects:
            inside = not inside
        j = i

    return inside


def point_to_segment_distance(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1

    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - x1, py - y1)

    c2 = vx * vx + vy * vy
    if c2 <= 1e-12:
        return math.hypot(px - x1, py - y1)

    t = max(0.0, min(1.0, c1 / c2))
    proj_x = x1 + t * vx
    proj_y = y1 + t * vy
    return math.hypot(px - proj_x, py - proj_y)


class VirtualFenceGuardV2(Node):
    def __init__(self) -> None:
        super().__init__("virtual_fence_guard_v2")

        self.cb_group = ReentrantCallbackGroup()

        # Topics / interfaces
        self.declare_parameter("load_waypoints_topic", "/load_waypoints")
        self.declare_parameter("navigation_active_topic", "/navigation_active")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("notifications_topic", "/agar_notifications")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("brake_topic", "/brake_status")
        self.declare_parameter("fence_path_topic", "/virtual_fence_path")
        self.declare_parameter("fromll_service", "/fromLL")

        # Frames / sources
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("pose_source", "auto")  # auto | tf | gps

        # Behavior
        self.declare_parameter("waypoints_dir", "/home/nvidia/ros_data/waypoints")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("allow_entry_from_outside", True)
        self.declare_parameter("enforce_margin_stop", False)
        self.declare_parameter("fence_margin_m", 0.30)
        self.declare_parameter("publish_brake_on_violation", True)
        self.declare_parameter("brake_value", 1)
        self.declare_parameter("notification_cooldown_sec", 10.0)
        self.declare_parameter("margin_warn_cooldown_sec", 10.0)
        self.declare_parameter("fromll_timeout_sec", 1.0)

        self.load_waypoints_topic = self.get_parameter("load_waypoints_topic").value
        self.navigation_active_topic = self.get_parameter("navigation_active_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.notifications_topic = self.get_parameter("notifications_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.brake_topic = self.get_parameter("brake_topic").value
        self.fence_path_topic = self.get_parameter("fence_path_topic").value
        self.fromll_service = self.get_parameter("fromll_service").value

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.pose_source = str(self.get_parameter("pose_source").value).strip().lower()

        self.waypoints_dir = self.get_parameter("waypoints_dir").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.allow_entry_from_outside = bool(self.get_parameter("allow_entry_from_outside").value)
        self.enforce_margin_stop = bool(self.get_parameter("enforce_margin_stop").value)
        self.fence_margin_m = float(self.get_parameter("fence_margin_m").value)
        self.publish_brake_on_violation = bool(self.get_parameter("publish_brake_on_violation").value)
        self.brake_value = int(self.get_parameter("brake_value").value)
        self.notification_cooldown_sec = float(self.get_parameter("notification_cooldown_sec").value)
        self.margin_warn_cooldown_sec = float(self.get_parameter("margin_warn_cooldown_sec").value)
        self.fromll_timeout_sec = float(self.get_parameter("fromll_timeout_sec").value)

        # Runtime state
        self.navigation_active = False
        self.entered_fence_once = False
        self.violation_active = False
        self.last_notification_ts = 0.0
        self.last_margin_warn_ts = 0.0

        self.mission_csv = ""
        self.fence_csv = ""
        self.fence_ll: List[Tuple[float, float]] = []
        self.fence_map: List[Tuple[float, float]] = []

        self.gps_lat: Optional[float] = None
        self.gps_lon: Optional[float] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fromll_client = self.create_client(FromLL, self.fromll_service, callback_group=self.cb_group)
        while not self.fromll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for {self.fromll_service} service...")
        self.fromll_req = FromLL.Request()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.brake_pub = self.create_publisher(Int16, self.brake_topic, 10)
        self.notifications_pub = self.create_publisher(String, self.notifications_topic, 10)
        self.nav_active_pub = self.create_publisher(Bool, self.navigation_active_topic, 10)
        self.fence_path_pub = self.create_publisher(Path, self.fence_path_topic, 10)

        self.create_subscription(String, self.load_waypoints_topic, self.on_load_waypoints, 10, callback_group=self.cb_group)
        self.create_subscription(Bool, self.navigation_active_topic, self.on_navigation_active, 10, callback_group=self.cb_group)
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps_fix, 10, callback_group=self.cb_group)

        period = 1.0 / max(0.1, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer, callback_group=self.cb_group)

        self.get_logger().info(
            "virtual_fence_guard_v2 started | pose_source=%s | map=%s robot=%s"
            % (self.pose_source, self.map_frame, self.robot_frame)
        )

    # -------------------------- ROS callbacks --------------------------

    def on_navigation_active(self, msg: Bool) -> None:
        prev = self.navigation_active
        self.navigation_active = bool(msg.data)

        if self.navigation_active and not prev:
            self.entered_fence_once = False
            self.violation_active = False
            self.get_logger().info("Navigation ON: fence-entry state reset.")

    def on_gps_fix(self, msg: NavSatFix) -> None:
        self.gps_lat = float(msg.latitude)
        self.gps_lon = float(msg.longitude)

    def on_load_waypoints(self, msg: String) -> None:
        mission = normalize_mission_filename(msg.data)
        if not mission:
            return

        self.entered_fence_once = False
        self.violation_active = False

        self.mission_csv = mission
        self.fence_csv = fence_filename_from_mission(mission)
        fence_path = os.path.join(self.waypoints_dir, self.fence_csv)

        if not os.path.exists(fence_path):
            self._reset_fence()
            self._notify(f"Virtual fence file not found: {self.fence_csv}")
            self.get_logger().warn(f"Fence file not found: {fence_path}")
            return

        self._load_fence_csv(fence_path)
        ok = self._build_map_polygon()
        if not ok:
            self._reset_fence()
            self._notify("Virtual fence conversion failed (/fromLL).")
            self.get_logger().error("Fence conversion failed via /fromLL")
            return

        self._notify(f"Virtual fence loaded: {self.fence_csv} ({len(self.fence_map)} pts)")
        self.get_logger().info(
            f"Fence loaded: {self.fence_csv} | points={len(self.fence_map)}"
        )

    # -------------------------- Fence IO --------------------------

    def _reset_fence(self) -> None:
        self.fence_ll = []
        self.fence_map = []

    def _load_fence_csv(self, fence_path: str) -> None:
        pts: List[Tuple[float, float]] = []
        with open(fence_path, "r", newline="") as f:
            for line in f:
                stripped = line.strip()
                if not stripped:
                    continue

                # Supports both 'lat|lon' and 'lat,lon'.
                if "|" in stripped:
                    parts = stripped.split("|")
                else:
                    parts = stripped.split(",")

                if len(parts) < 2:
                    continue

                try:
                    lat = float(parts[0].strip())
                    lon = float(parts[1].strip())
                except ValueError:
                    continue
                pts.append((lat, lon))

        cleaned: List[Tuple[float, float]] = []
        for p in pts:
            if not cleaned:
                cleaned.append(p)
                continue
            if abs(cleaned[-1][0] - p[0]) > 1e-12 or abs(cleaned[-1][1] - p[1]) > 1e-12:
                cleaned.append(p)

        self.fence_ll = cleaned

    def _build_map_polygon(self) -> bool:
        if len(self.fence_ll) < 3:
            self.fence_map = []
            return False

        poly: List[Tuple[float, float]] = []
        for lat, lon in self.fence_ll:
            xy = self._ll_to_map_xy(lat, lon)
            if xy is None:
                return False
            poly.append(xy)

        self.fence_map = poly
        return True

    def _ll_to_map_xy(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        p = GeoPoint()
        p.latitude = float(lat)
        p.longitude = float(lon)
        p.altitude = 0.0

        self.fromll_req.ll_point = p
        future = self.fromll_client.call_async(self.fromll_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.fromll_timeout_sec)

        if not future.done():
            return None

        res = future.result()
        if res is None:
            return None

        return (float(res.map_point.x), float(res.map_point.y))

    # -------------------------- Runtime checks --------------------------

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        # 1) TF pose in map frame
        if self.pose_source in ("tf", "auto"):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.robot_frame,
                    Time(),
                )
                return (
                    float(t.transform.translation.x),
                    float(t.transform.translation.y),
                )
            except TransformException:
                if self.pose_source == "tf":
                    return None

        # 2) GPS fallback converted to map
        if self.pose_source in ("gps", "auto"):
            if self.gps_lat is None or self.gps_lon is None:
                return None
            return self._ll_to_map_xy(self.gps_lat, self.gps_lon)

        return None

    def _distance_to_fence_edge(self, x: float, y: float) -> float:
        if len(self.fence_map) < 2:
            return float("inf")

        d_min = float("inf")
        n = len(self.fence_map)
        for i in range(n):
            x1, y1 = self.fence_map[i]
            x2, y2 = self.fence_map[(i + 1) % n]
            d = point_to_segment_distance(x, y, x1, y1, x2, y2)
            if d < d_min:
                d_min = d
        return d_min

    def _publish_fence_path(self) -> None:
        if not self.fence_map:
            return

        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.fence_map:
            p = PoseStamped()
            p.header.frame_id = self.map_frame
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0
            msg.poses.append(p)

        x0, y0 = self.fence_map[0]
        p0 = PoseStamped()
        p0.header.frame_id = self.map_frame
        p0.pose.position.x = float(x0)
        p0.pose.position.y = float(y0)
        p0.pose.position.z = 0.0
        msg.poses.append(p0)

        self.fence_path_pub.publish(msg)

    def _stop_motion(self) -> None:
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

        if self.publish_brake_on_violation:
            b = Int16()
            b.data = int(self.brake_value)
            self.brake_pub.publish(b)

    def _deactivate_navigation(self, reason: str) -> None:
        self._stop_motion()

        nav = Bool()
        nav.data = False
        self.nav_active_pub.publish(nav)

        now = time.monotonic()
        if (now - self.last_notification_ts) >= self.notification_cooldown_sec:
            self.last_notification_ts = now
            self._notify(reason)

        self.get_logger().error(reason)

    def _notify(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.notifications_pub.publish(msg)

    def on_timer(self) -> None:
        self._publish_fence_path()

        if not self.navigation_active:
            return

        if len(self.fence_map) < 3:
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return

        rx, ry = robot_xy
        inside = point_in_polygon(rx, ry, self.fence_map)

        if inside:
            self.entered_fence_once = True

            if self.enforce_margin_stop:
                d_edge = self._distance_to_fence_edge(rx, ry)
                if d_edge <= self.fence_margin_m:
                    self.violation_active = True
                    self._deactivate_navigation(
                        "FENCE VIOLATION: robot is inside but too close to fence edge "
                        f"(d={d_edge:.2f}m <= {self.fence_margin_m:.2f}m)"
                    )
                    return
            else:
                d_edge = self._distance_to_fence_edge(rx, ry)
                now = time.monotonic()
                if d_edge <= self.fence_margin_m and (now - self.last_margin_warn_ts) >= self.margin_warn_cooldown_sec:
                    self.last_margin_warn_ts = now
                    self.get_logger().warn(
                        f"Fence margin warning: d={d_edge:.2f}m <= {self.fence_margin_m:.2f}m"
                    )
            return

        # Outside polygon
        if self.allow_entry_from_outside and not self.entered_fence_once:
            # Robot may start outside and approach entry path.
            return

        self.violation_active = True
        self._deactivate_navigation("FENCE VIOLATION: robot exited virtual fence polygon")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualFenceGuardV2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
