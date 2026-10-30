import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_slam = LaunchConfiguration('enable_slam')
    pkg_localization = get_package_share_directory('agar_slam')
    zed_odom_config = os.path.join(
        pkg_localization,
        'config',
        'localization_zed_odom_slam.yaml',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[zed_odom_config],
    )

    map_to_odom = Node(
        condition=UnlessCondition(enable_slam),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    )

    camera_cloud_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_laser',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'zedx_left_camera_frame', 'camera_laserscan'],
    )

    camera_cloud_to_laserscan = Node(
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/zedx/zed_node/point_cloud/cloud_registered'),
            ('scan', '/zedx/zed_node/scan'),
        ],
        parameters=[{
            'target_frame': 'camera_laserscan',
            'transform_tolerance': 0.01,
            'min_height': -0.9,
            'max_height': 0.3,
            'angle_min': -0.523,
            'angle_max': 0.523,
            'angle_increment': 0.0087,
            'time_increment': 0.1,
            'scan_time': 0.2,
            'range_min': 0.5,
            'range_max': 7.0,
            'use_inf': True,
            'use_sim_time': False,
        }],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_localization, 'launch'),
            '/agar_slam_launch.py',
        ]),
        condition=IfCondition(enable_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable slam_toolbox and let SLAM own map->odom.',
        ),
        ekf_node,
        map_to_odom,
        camera_cloud_to_laser_tf,
        camera_cloud_to_laserscan,
        slam_launch,
    ])
