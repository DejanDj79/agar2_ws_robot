import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    pkg_slam = get_package_share_directory('agar_slam')
    amcl_config = os.path.join(
        pkg_slam,
        'config',
        'amcl_velodyne_config.yaml',
    )

    zed_base_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_slam, 'launch'),
            '/agar_zed_base_odometry.launch.py',
        ]),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_config,
            {'yaml_filename': map_file},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/nvidia/workspace/maps/active_map.yaml',
            description='Full path to the map YAML file to load.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate map_server and amcl.',
        ),
        zed_base_odometry,
        map_server,
        amcl,
        lifecycle_manager,
    ])
