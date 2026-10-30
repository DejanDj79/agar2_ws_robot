import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Optional SLAM mode for production bringup.
# Keep exactly one SLAM_MODE line uncommented.
SLAM_MODE = 'off'
# SLAM_MODE = 'mapping'
# SLAM_MODE = 'localization'  # AMCL on .yaml/.pgm map
# SLAM_MODE = 'localization_slam_toolbox'
# Used only when SLAM_MODE is 'mapping'. Keep the current online behavior unless
# a controlled A/B test explicitly selects the standard profile.
SLAM_MAPPING_PROFILE = 'zed_constrained'
# SLAM_MAPPING_PROFILE = 'standard'
SLAM_MAP = '/home/nvidia/workspace/maps/active_map.yaml'
SLAM_POSEGRAPH = '/home/nvidia/workspace/maps/active_posegraph'
SLAM_AUTOSTART = 'true'
SLAM_START_VELODYNE = 'true'
# Passive test output only. When enabled in mapping mode, /scan_zed is recorded
# beside the Velodyne /scan but never connected to the online SLAM node.
SLAM_ENABLE_ZED_SCAN_PROBE = 'false'


def generate_launch_description():
    enable_system_health = LaunchConfiguration('enable_system_health')
    enable_slam = LaunchConfiguration('enable_slam')
    system_health_config_file = LaunchConfiguration('system_health_config_file')
    system_health_check_period = LaunchConfiguration('system_health_check_period')
    system_health_startup_grace = LaunchConfiguration('system_health_startup_grace')
    enable_system_health_notifications = LaunchConfiguration('enable_system_health_notifications')
    system_health_notification_topic = LaunchConfiguration('system_health_notification_topic')
    system_health_notification_repeat_sec = LaunchConfiguration('system_health_notification_repeat_sec')
    system_health_notify_on_warn = LaunchConfiguration('system_health_notify_on_warn')
    enable_system_health_rviz = LaunchConfiguration('enable_system_health_rviz')
    system_health_rviz_frame = LaunchConfiguration('system_health_rviz_frame')

    runtime_report_health_file = os.path.expanduser('~/system_health/system_health_real_robot.yaml')
    default_system_health_config = (
        runtime_report_health_file
        if os.path.exists(runtime_report_health_file)
        else os.path.join(
            get_package_share_directory('agar_bringup'),
            'config',
            'system_health_jetson.yaml',
        )
    )
    zed_object_detection_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'zed_object_detection.yaml',
    )
    zed_slam_overrides = os.path.join(
        get_package_share_directory('agar_slam'),
        'config',
        'zed_slam_overrides.yaml',
    )
    fuzzy_startup_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'fuzzy_startup_params.yaml',
    )
    slam_health_config = os.path.join(
        get_package_share_directory('agar_slam'),
        'config',
        'slam_health_monitor.yaml',
    )
    
    description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_description'), 'launch'),
         '/agar_description_launch.py'])
      )
    
    localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_localization'), 'launch'),
         '/agar_localization_launch.py']),
         launch_arguments={
             'enable_slam': enable_slam,
         }.items()
      )

    slam_modes_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(PathJoinSubstitution([
         FindPackageShare('agar_slam'),
         'launch',
         'agar_slam_modes.launch.py',
      ])),
      launch_arguments={
          'mode': SLAM_MODE,
          'mapping_profile': SLAM_MAPPING_PROFILE,
          'map': SLAM_MAP,
          'posegraph': SLAM_POSEGRAPH,
          'autostart': SLAM_AUTOSTART,
          'enable_zed': 'false',
          'enable_description': 'false',
          'enable_velodyne': SLAM_START_VELODYNE,
          'enable_zed_scan_probe': SLAM_ENABLE_ZED_SCAN_PROBE,
      }.items()
    )

    if SLAM_MODE in ('mapping', 'localization', 'localization_amcl', 'localization_slam_toolbox'):
        localization_actions = [
            LogInfo(msg=f'Optional SLAM mode enabled from agar_jetson_launch.py: {SLAM_MODE}'),
            slam_modes_launch,
        ]
    elif SLAM_MODE == 'off':
        localization_actions = [localization_launch]
    else:
        localization_actions = [
            LogInfo(msg='Invalid SLAM_MODE in agar_jetson_launch.py. Falling back to normal localization.'),
            localization_launch,
        ]
    
    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_navigation'), 'launch'),
         '/agar_navigation_launch.py'])
      )

    waypoint_follower = Node(
      package='agar_scripts', 
      executable='waypoint_follower',
      output='screen',
      remappings=[
          ('/cmd_vel', '/cmd_vel_raw'),
      ],
    )

    fuzzy_middleware = Node(
        package='agar_scripts',
        executable='fuzzy_middleware',
        output='screen',
        respawn=True,
        parameters=[fuzzy_startup_config],
    )

    virtual_fence = Node(
        package='agar_scripts',
        executable='virtual_fence',
        output='screen',
        respawn=True,
        parameters=[{
            'waypoints_dir': '/home/nvidia/ros_data/waypoints',
            'map_frame': 'map',
            'robot_frame': 'base_footprint',
            'pose_source': 'auto',
            'allow_entry_from_outside': True,
            'enforce_margin_stop': False,
            'fence_margin_m': 0.30,
            'publish_brake_on_violation': True,
        }],
    )

    toknav = Node(
      package='agar_scripts', 
      executable='toknav_gps',
      output='screen',
      respawn=True
    )

    datum_correction = Node(
      package='agar_scripts', 
      executable='datum_correction',
      output='screen',
      respawn=True
    )

    waypoint_service = Node(
      package='agar_scripts', 
      executable='waypoint_service',
      output='screen',
      respawn=True
    )

    waypoint_files_service = Node(
      package='agar_scripts', 
      executable='waypoint_files_service',
      output='screen',
      respawn=True
    )

    get_mission_editor_service = Node(
      package='agar_scripts', 
      executable='get_mission_editor_service',
      output='screen',
      respawn=True
    )

    set_mission_editor_service = Node(
      package='agar_scripts', 
      executable='set_mission_editor_service',
      output='screen',
      respawn=True
    )
     
    position_data_service = Node(
      package='agar_scripts', 
      executable='position_data_service',
      output='screen',
      respawn=True
    )

    slam_map_manager = Node(
      package='agar_slam',
      executable='slam_map_manager',
      output='screen',
      respawn=True
    )

    slam_health_monitor = Node(
      package='agar_slam',
      executable='slam_health_monitor',
      output='screen',
      respawn=True,
      respawn_delay=2.0,
      parameters=[
          slam_health_config,
          {'mode': SLAM_MODE},
      ],
    )

    object_detection = Node(
      package='agar_scripts', 
      executable='object_detection',
      output='screen',
      respawn=True
    )

    camera_mount_position = Node(
      package='agar_scripts', 
      executable='camera_mount_position',
      output='screen',
      respawn=True
    )

    collision_checker = Node(
      package='agar_scripts', 
      executable='collision_checker',
      output='screen',
      respawn=True
    )

    battery_status = Node(
      package='agar_scripts', 
      executable='battery_status',
      output='screen',
      respawn=True
    )

    jetson_online = Node(
      package='agar_scripts', 
      executable='jetson_status_pub',
      output='screen',
      respawn=True
    )

    system_health_monitor = Node(
      package='system_health_tools',
      executable='system_health_monitor',
      output='screen',
      respawn=True,
      parameters=[{
          'config_file': system_health_config_file,
          'check_period_sec': system_health_check_period,
          'startup_grace_sec': system_health_startup_grace,
          'notifications_enabled': enable_system_health_notifications,
          'notification_topic': system_health_notification_topic,
          'notification_repeat_sec': system_health_notification_repeat_sec,
          'notify_on_warn': system_health_notify_on_warn,
      }],
      condition=IfCondition(enable_system_health)
    )

    system_health_rviz = Node(
      package='system_health_tools',
      executable='system_health_rviz',
      output='screen',
      respawn=True,
      parameters=[{
          'frame_id': system_health_rviz_frame,
          'max_items': 80,
          'anchor_x': 2.0,
          'anchor_y': -1.0,
          'anchor_z': 1.8,
          'line_spacing': 0.20,
          'text_height': 0.12,
          'show_ok_items': True,
          'show_messages': False,
          'use_colors': False,
      }],
      condition=IfCondition(enable_system_health_rviz)
    )

    zed_front_launch_arguments = {
        'camera_model': 'zedx',
        'camera_name': 'zedx',
        'serial_number': '45931755',
        'object_detection_config_path': zed_object_detection_config,
    }
    if SLAM_MODE in (
        'mapping',
        'localization',
        'localization_amcl',
        'localization_slam_toolbox',
    ):
        # agar_slam owns odom -> base_footprint in SLAM modes. Preserve the
        # ZED wrapper defaults in off mode so normal operation is unchanged.
        zed_front_launch_arguments.update({
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
            'ros_params_override_path': zed_slam_overrides,
        })

    zedcamera_front_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(PathJoinSubstitution([
         FindPackageShare('zed_wrapper'),
         'launch',
         'zed_camera.launch.py',
      ])),
          launch_arguments=zed_front_launch_arguments.items()
      )
    
    zedcamera_rear_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(PathJoinSubstitution([
         FindPackageShare('zed_wrapper'),
         'launch',
         'zed_camera.launch.py',
      ])),
          launch_arguments={
              'camera_model': 'zedxm',
              'camera_name': 'zedxm',
              'serial_number': '55327043',
              'node_log_type': 'screen',
              'publish_tf': 'false',
              'publish_map_tf': 'false',
              'publish_imu_tf': 'false',
          }.items()
      )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Legacy ZED-scan SLAM switch used only when SLAM_MODE is off. Prefer SLAM_MODE=mapping.',
        ),
        DeclareLaunchArgument(
            'enable_system_health',
            default_value='true',
            description='Start runtime system health monitor and publish /diagnostics.',
        ),
        DeclareLaunchArgument(
            'system_health_config_file',
            default_value=default_system_health_config,
            description='Path to system health YAML contract file.',
        ),
        DeclareLaunchArgument(
            'system_health_check_period',
            default_value='1.0',
            description='System health check period in seconds.',
        ),
        DeclareLaunchArgument(
            'system_health_startup_grace',
            default_value='10.0',
            description='Grace period (sec) before stale/no-message checks become errors.',
        ),
        DeclareLaunchArgument(
            'enable_system_health_notifications',
            default_value='true',
            description='Publish health alert/recovery events to /agar_notifications.',
        ),
        DeclareLaunchArgument(
            'system_health_notification_topic',
            default_value='/agar_notifications',
            description='Topic for system health alert text notifications.',
        ),
        DeclareLaunchArgument(
            'system_health_notification_repeat_sec',
            default_value='300.0',
            description='Repeat period (sec) for unchanged active health failures.',
        ),
        DeclareLaunchArgument(
            'system_health_notify_on_warn',
            default_value='true',
            description='If true, publish notifications for WARN and ERROR checks.',
        ),
        DeclareLaunchArgument(
            'enable_system_health_rviz',
            default_value='true',
            description='Render system health summary/checks in RViz via MarkerArray.',
        ),
        DeclareLaunchArgument(
            'system_health_rviz_frame',
            default_value='odom',
            description='Frame used for RViz system health markers.',
        ),
        toknav,
        camera_mount_position,
        description_launch,
        *localization_actions,
        navigation_launch,
        waypoint_follower,

        object_detection,
        # collision_checker,
        # Disabled until the installed dalybms package provides the
        # DalyBMSBluetooth API expected by battery_status.py.
        # battery_status,
        jetson_online,
        fuzzy_middleware,
        virtual_fence,

        waypoint_service,
        waypoint_files_service,
        get_mission_editor_service,
        set_mission_editor_service,
        position_data_service,
        slam_map_manager,
        slam_health_monitor,
        system_health_monitor,
        system_health_rviz,

        zedcamera_front_launch,
        # zedcamera_rear_launch,
    ])
    
