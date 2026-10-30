"""Pure calculations shared by the online SLAM health monitor."""

import math


def normalize_angle(angle):
    """Normalize an angle to the shortest signed rotation."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Extract planar yaw from a ROS quaternion-like object."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def planar_step(previous, current):
    """Return planar translation and absolute yaw steps."""
    translation = math.hypot(
        current[0] - previous[0],
        current[1] - previous[1],
    )
    yaw = abs(math.degrees(normalize_angle(current[2] - previous[2])))
    return translation, yaw


def expected_engine_nodes(mode):
    """Return the localization nodes required by a configured mode."""
    if mode in ('mapping', 'localization_slam_toolbox'):
        return ('/slam_toolbox',)
    if mode in ('localization', 'localization_amcl'):
        return ('/amcl', '/map_server')
    return ()


def sample_rate_hz(received_times):
    """Estimate a message rate from monotonic reception timestamps."""
    if len(received_times) < 2:
        return 0.0
    duration = received_times[-1] - received_times[0]
    if duration <= 0.0:
        return 0.0
    return float(len(received_times) - 1) / duration
