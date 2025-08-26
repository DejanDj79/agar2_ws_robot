import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

  use_sim_time_config = os.path.join(
    get_package_share_directory('agar_bringup'),
    'config',
    'sim_time.yaml'
  )
 
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_agar_gazebo = FindPackageShare(package='agar_gazebo').find('agar_gazebo')
 
  world_file_name = 'agar_empty.world'
  # world_file_name = 'agar_cones.world'
  # world_file_name = 'agar_walls.world'
  # world_file_name = 'agar_vineyard.world'
  # world_file_name = 'agar_village.world'
  world_path = os.path.join(pkg_agar_gazebo, 'worlds', world_file_name)

 
  headless = LaunchConfiguration('headless')
  use_simulator = LaunchConfiguration('use_simulator')
  
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='True',
    description='Whether to execute gzclient')
     
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  world = LaunchConfiguration('world')
    
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items(),
  )
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
  )
 
  # Spawn AgAR model    
  spawn_entity_node = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'AgAR'],
    parameters=[use_sim_time_config],
    output='screen'
  )

  load_joint_state_broadcaster = ExecuteProcess(
      cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
      output='screen'
  )

  load_diff_drive_base_controller = ExecuteProcess(
      cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_base_controller'],
      output='screen'
  )

  # transform_node = Node(
  #     package="tf2_ros",
  #     executable="static_transform_publisher",
  #     arguments = ["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "map", "--child-frame-id", "odom"]
  # )

  ld = LaunchDescription()
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  # ld.add_action(start_gazebo_server_cmd)
  # ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_node)
  # ld.add_action(robot_steering_node)

  ld.add_action(load_joint_state_broadcaster)
  ld.add_action(load_diff_drive_base_controller)

  # ld.add_action(transform_node)

  return ld