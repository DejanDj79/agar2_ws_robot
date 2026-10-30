import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mapping_slam_actions(context, pkg_slam):
    enable_slam = LaunchConfiguration('enable_slam')
    mapping_profile = LaunchConfiguration('mapping_profile').perform(context)
    profile_configs = {
        'standard': 'slamtoolbox_velodyne_config.yaml',
        'zed_constrained': 'slamtoolbox_velodyne_zed_constrained_config.yaml',
    }
    config_filename = profile_configs.get(mapping_profile)
    if config_filename is None:
        raise RuntimeError(
            'Unsupported mapping_profile: {}. Expected one of: {}'.format(
                mapping_profile,
                ', '.join(sorted(profile_configs)),
            )
        )

    slam_config = os.path.join(pkg_slam, 'config', config_filename)
    return [
        LogInfo(
            condition=IfCondition(enable_slam),
            msg='SLAM mapping profile: {} ({})'.format(
                mapping_profile,
                slam_config,
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_slam, 'launch'),
                '/agar_slam_velodyne_launch.py',
            ]),
            condition=IfCondition(enable_slam),
            launch_arguments={
                'slam_params_file': slam_config,
            }.items(),
        ),
    ]


def generate_launch_description():
    enable_slam = LaunchConfiguration('enable_slam')
    pkg_slam = get_package_share_directory('agar_slam')

    zed_base_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_slam, 'launch'),
            '/agar_zed_base_odometry.launch.py',
        ]),
    )

    map_to_odom = Node(
        condition=UnlessCondition(enable_slam),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable slam_toolbox with Velodyne /scan input.',
        ),
        DeclareLaunchArgument(
            'mapping_profile',
            default_value='zed_constrained',
            choices=['standard', 'zed_constrained'],
            description=(
                'Select the standard baseline or constrained ZED odometry '
                'mapping profile.'
            ),
        ),
        zed_base_odometry,
        map_to_odom,
        OpaqueFunction(
            function=mapping_slam_actions,
            args=[pkg_slam],
        ),
    ])
