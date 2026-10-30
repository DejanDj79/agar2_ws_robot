from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    mapping_profile = LaunchConfiguration('mapping_profile')
    map_file = LaunchConfiguration('map')
    posegraph_file = LaunchConfiguration('posegraph')
    autostart = LaunchConfiguration('autostart')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_velodyne = LaunchConfiguration('enable_velodyne')
    enable_description = LaunchConfiguration('enable_description')
    enable_zed_scan_probe = LaunchConfiguration('enable_zed_scan_probe')

    pkg_description = FindPackageShare('agar_description')
    pkg_slam = FindPackageShare('agar_slam')
    pkg_zed_wrapper = FindPackageShare('zed_wrapper')
    pkg_velodyne = FindPackageShare('velodyne')

    is_off = PythonExpression(["'", mode, "' == 'off'"])
    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_amcl_localization = PythonExpression([
        "'", mode, "' in ['localization', 'localization_amcl']"
    ])
    is_slam_toolbox_localization = PythonExpression([
        "'", mode, "' == 'localization_slam_toolbox'"
    ])
    is_invalid_mode = PythonExpression([
        "'", mode, "' not in ['off', 'mapping', 'localization', 'localization_amcl', 'localization_slam_toolbox']"
    ])

    description_enabled = PythonExpression([
        "'", enable_description,
        "' == 'true' and '", mode,
        "' in ['mapping', 'localization', 'localization_amcl', 'localization_slam_toolbox']"
    ])
    zed_enabled = PythonExpression([
        "'", enable_zed,
        "' == 'true' and '", mode,
        "' in ['mapping', 'localization', 'localization_amcl', 'localization_slam_toolbox']"
    ])
    velodyne_enabled = PythonExpression([
        "'", enable_velodyne,
        "' == 'true' and '", mode,
        "' in ['mapping', 'localization', 'localization_amcl', 'localization_slam_toolbox']"
    ])
    zed_scan_probe_enabled = PythonExpression([
        "'", enable_zed_scan_probe,
        "' == 'true' and '", mode,
        "' == 'mapping'"
    ])

    zed_object_detection_config = PathJoinSubstitution([
        FindPackageShare('agar_bringup'),
        'config',
        'zed_object_detection.yaml',
    ])
    zed_slam_overrides = PathJoinSubstitution([
        pkg_slam,
        'config',
        'zed_slam_overrides.yaml',
    ])

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_description,
            'launch',
            'agar_description_launch.py',
        ])),
        condition=IfCondition(description_enabled),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_slam,
            'launch',
            'agar_localization_zed_odom_velodyne_slam_launch.py',
        ])),
        condition=IfCondition(is_mapping),
        launch_arguments={
            'enable_slam': 'true',
            'mapping_profile': mapping_profile,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_slam,
            'launch',
            'agar_localization_zed_odom_velodyne_amcl_launch.py',
        ])),
        condition=IfCondition(is_amcl_localization),
        launch_arguments={
            'map': map_file,
            'autostart': autostart,
        }.items(),
    )

    slam_toolbox_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_slam,
            'launch',
            'agar_localization_zed_odom_velodyne_slam_toolbox_launch.py',
        ])),
        condition=IfCondition(is_slam_toolbox_localization),
        launch_arguments={
            'posegraph': posegraph_file,
        }.items(),
    )

    zedcamera_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_zed_wrapper,
            'launch',
            'zed_camera.launch.py',
        ])),
        condition=IfCondition(zed_enabled),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zedx',
            'serial_number': '45931755',
            'object_detection_config_path': zed_object_detection_config,
            'ros_params_override_path': zed_slam_overrides,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
        }.items(),
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_velodyne,
            'launch',
            'velodyne-all-nodes-VLP16-launch.py',
        ])),
        condition=IfCondition(velodyne_enabled),
    )

    zed_scan_probe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_slam,
            'launch',
            'agar_zed_scan_probe.launch.py',
        ])),
        condition=IfCondition(zed_scan_probe_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='off',
            description='SLAM mode: off, mapping, or localization.',
        ),
        DeclareLaunchArgument(
            'mapping_profile',
            default_value='zed_constrained',
            choices=['standard', 'zed_constrained'],
            description=(
                'Mapping-only SLAM Toolbox profile. Ignored in every other mode.'
            ),
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/nvidia/workspace/maps/active_map.yaml',
            description='Map YAML used when mode:=localization or mode:=localization_amcl.',
        ),
        DeclareLaunchArgument(
            'posegraph',
            default_value='/home/nvidia/workspace/maps/active_posegraph',
            description='Posegraph base path without .posegraph suffix for mode:=localization_slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate map_server and amcl in localization mode.',
        ),
        DeclareLaunchArgument(
            'enable_zed',
            default_value='true',
            description='Start the front ZEDX camera unless another launch already started it.',
        ),
        DeclareLaunchArgument(
            'enable_velodyne',
            default_value='true',
            description='Start Velodyne VLP16 nodes unless another launch already started them.',
        ),
        DeclareLaunchArgument(
            'enable_description',
            default_value='true',
            description='Start robot description unless another launch already started it.',
        ),
        DeclareLaunchArgument(
            'enable_zed_scan_probe',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish passive /scan_zed in mapping mode for a same-drive '
                'offline comparison. It never feeds the online SLAM node.'
            ),
        ),
        LogInfo(
            condition=IfCondition(is_off),
            msg='SLAM mode is off: no SLAM, AMCL, ZED, Velodyne, or robot description nodes are started.',
        ),
        LogInfo(
            condition=IfCondition(is_invalid_mode),
            msg='Invalid SLAM mode. Use off, mapping, localization/localization_amcl, or localization_slam_toolbox.',
        ),
        description_launch,
        mapping_launch,
        localization_launch,
        slam_toolbox_localization_launch,
        zedcamera_front_launch,
        velodyne_launch,
        zed_scan_probe_launch,
    ])
