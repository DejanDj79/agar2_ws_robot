import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    enable_zed = LaunchConfiguration('enable_zed')
    enable_velodyne = LaunchConfiguration('enable_velodyne')
    map_file = LaunchConfiguration('map')

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

    amcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('agar_slam'), 'launch'),
            '/agar_localization_zed_odom_velodyne_amcl_launch.py',
        ]),
        launch_arguments={
            'map': map_file,
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

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('velodyne'), 'launch'),
            '/velodyne-all-nodes-VLP16-launch.py',
        ]),
        condition=IfCondition(enable_velodyne),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_zed',
            default_value='true',
            description='Start the front ZEDX camera for ZED odometry.',
        ),
        DeclareLaunchArgument(
            'enable_velodyne',
            default_value='true',
            description='Start Velodyne VLP16 nodes for AMCL /scan input.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/nvidia/workspace/maps/slam_lab_test.yaml',
            description='Full path to the map YAML file to load.',
        ),
        description_launch,
        amcl_localization_launch,
        zedcamera_front_launch,
        velodyne_launch,
    ])
