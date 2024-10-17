import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
 
def generate_launch_description():
 
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_agar_gazebo = FindPackageShare(package='agar_gazebo').find('agar_gazebo')
 
  world_file_name = 'agar_empty.world'
  world_path = os.path.join(pkg_agar_gazebo, 'worlds', world_file_name)
   
  # Set the path to the SDF model files.
  # gazebo_models_path = os.path.join(pkg_agar_gazebo, 'models')
  # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
 
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
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
    output='screen'
  )

  robot_steering_node = Node(
    package='rqt_robot_steering', 
    executable='rqt_robot_steering',
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

  ld = LaunchDescription()
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_node)
  # ld.add_action(robot_steering_node)

  ld.add_action(load_joint_state_broadcaster)
  ld.add_action(load_diff_drive_base_controller)

  return ld