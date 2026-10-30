import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    enable_slam = LaunchConfiguration('enable_slam')
    enable_zed = LaunchConfiguration('enable_zed')

    zed_object_detection_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'zed_object_detection.yaml',
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('agar_description'), 'launch'),
            '/agar_description_launch.py',
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('agar_localization'), 'launch'),
            '/agar_localization_launch.py',
        ]),
        launch_arguments={
            'enable_slam': enable_slam,
        }.items(),
    )

    zedcamera_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py',
        ]),
        condition=IfCondition(enable_zed),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zedx',
            'serial_number': '45931755',
            'object_detection_config_path': zed_object_detection_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_zed',
            default_value='true',
            description='Start only the front ZEDX camera for lab SLAM/sensor testing.',
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable slam_toolbox. Keep false for first lab sensor-only test.',
        ),
        description_launch,
        localization_launch,
        zedcamera_front_launch,
    ])
