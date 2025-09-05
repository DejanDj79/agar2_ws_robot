import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource 

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # declared_arguments = []

    gazebo_config_file = os.path.join(
        get_package_share_directory('agar_gazebo'),
        'config',
        'gazebo.config'
    )

    gazebo_bridge_config_file = os.path.join(
        get_package_share_directory('agar_gazebo'),
        'config',
        'gazebo_bridge.yaml'
    )

    world_path = os.path.join(
        get_package_share_directory('agar_gazebo'),
        'worlds',
        'agar_world_full.sdf'
    )
    
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros_gz_sim'), 'launch'),
         '/gz_sim.launch.py']),
          launch_arguments={
              'gz_args': f'{world_path} -r --gui-config {gazebo_config_file}' #'empty.sdf -r' # -s -silent mode
          }.items()
      )
    
    ros_gz_sim = Node(
        package='ros_gz_sim', 
        executable='create',
        # output='screen',
        respawn=True,
        arguments=['-topic', 'robot_description']
    ) 

    ros_gz_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        respawn=True,
        parameters=[
            {'config_file': gazebo_bridge_config_file},
        ]
    ) 

    return LaunchDescription([
        gazebo_launch,
        ros_gz_sim,
        ros_gz_bridge,
    ])