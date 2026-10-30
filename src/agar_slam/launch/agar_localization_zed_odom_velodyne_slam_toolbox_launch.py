import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    posegraph = LaunchConfiguration('posegraph')
    pkg_slam = get_package_share_directory('agar_slam')
    slam_toolbox_config = os.path.join(
        pkg_slam,
        'config',
        'slamtoolbox_velodyne_localization_config.yaml',
    )

    zed_base_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_slam, 'launch'),
            '/agar_zed_base_odometry.launch.py',
        ]),
    )

    slam_toolbox_localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config,
            {'map_file_name': posegraph},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'posegraph',
            default_value='/home/nvidia/workspace/maps/active_posegraph',
            description='Posegraph base path without .posegraph suffix.',
        ),
        zed_base_odometry,
        slam_toolbox_localization,
    ])
