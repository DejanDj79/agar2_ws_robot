from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    config_file = PathJoinSubstitution([
        FindPackageShare('agar_slam'),
        'config',
        'zed_scan_probe.yaml',
    ])

    zed_scan_probe = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='zed_scan_probe',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', scan_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/zedx/zed_node/point_cloud/cloud_registered',
            description='Registered ZED PointCloud2 input used only by the probe.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_zed',
            description='Passive ZED-derived LaserScan output for bag recording.',
        ),
        LogInfo(msg=[
            'Passive ZED scan probe enabled: ',
            cloud_topic,
            ' -> ',
            scan_topic,
            '. SLAM Toolbox remains connected to /scan.',
        ]),
        zed_scan_probe,
    ])
