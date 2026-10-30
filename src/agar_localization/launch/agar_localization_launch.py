import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  pkg_localization = FindPackageShare(package='agar_localization').find('agar_localization')
  robot_localization_file_path = os.path.join(pkg_localization, 'config', 'localization_config.yaml') 
  enable_slam = LaunchConfiguration('enable_slam')

  declare_enable_slam = DeclareLaunchArgument(
    'enable_slam',
    default_value='false',
    description='Enable slam_toolbox and let SLAM own map->odom. If false, publish static map->odom for GPS/localization mode.'
  )

  gnss_fix_to_odometry_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[robot_localization_file_path], #, use_sim_time_config
        # remappings=[
        #     ("imu/data", "imu/data"),
        #     ("gps/fix", "gps/fix"),                # IN
        #     ("odometry/gps", "odometry/gps"),       # OUT
        # ],
    )

  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path], #, use_sim_time_config
    # remappings=[
    #       ("imu/data", "imu/data"),
    #   ],    
    )
  
  map_to_odom = Node(
      condition=UnlessCondition(enable_slam),
      package="tf2_ros",
      executable="static_transform_publisher",
      name="odom_to_map",
      output="screen",
      arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
  )
  
  camera_cloud_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_laser",
        output="screen",
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'zedx_left_camera_frame', 'camera_laserscan']
    )

  camera_cloud_to_laserscan = Node(
      name='pointcloud_to_laserscan',
      package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
      remappings=[('cloud_in', '/zedx/zed_node/point_cloud/cloud_registered'),
                  ('scan', '/zedx/zed_node/scan')],
      parameters=[{
          'target_frame': 'camera_laserscan',
          'transform_tolerance': 0.01,
          'min_height': -0.9,
          'max_height': 0.3,
          'angle_min': -0.523, 
          'angle_max': 0.523,
          'angle_increment': 0.0087,
          'time_increment': 0.1,
          'scan_time': 20.0,
          'range_min': 0.5,
          'range_max': 7.0,
          'use_inf': True,
          'use_sime_time': False,
      }],
  )

  slam_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_slam'), 'launch'),
         '/agar_slam_launch.py']),
      condition=IfCondition(enable_slam),
      )
  
  ld = LaunchDescription()
  ld.add_action(declare_enable_slam)
  
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(gnss_fix_to_odometry_node)
  
  ld.add_action(map_to_odom)

  ld.add_action(camera_cloud_to_laser_tf)
  ld.add_action(camera_cloud_to_laserscan)

  ld.add_action(slam_launch)

  # ld.add_action(odom_to_basefootprint)

  # ld.add_action(pointcloud_to_laserscan)

  return ld