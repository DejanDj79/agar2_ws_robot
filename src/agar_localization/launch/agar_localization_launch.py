import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  pkg_localization = FindPackageShare(package='agar_localization').find('agar_localization')
  robot_localization_file_path = os.path.join(pkg_localization, 'config', 'localization_config.yaml') 
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  gnss_fix_to_odometry_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[robot_localization_file_path, 
            {"use_sim_time": use_sim_time}],
        # remappings=[
        #     ("gps/fix", "/ublox_gps_node/fix"),     # IN
        #     ("odometry/gps", "odometry/gps"),       # OUT
        # ],
    )

  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
    # remappings=[
    #         ("odometry/filtered", "odometry/agar_odom"),
    #     ],    
    )
  

  map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_map",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    )
  
  odom_to_basefootprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_map",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint'],
    )

  ld = LaunchDescription()
  
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(gnss_fix_to_odometry_node)
  
  ld.add_action(map_to_odom)
  ld.add_action(odom_to_basefootprint)

  return ld