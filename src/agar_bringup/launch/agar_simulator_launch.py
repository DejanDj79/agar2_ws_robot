import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('agar_bringup'),
      'config',
      'agar_simulator.yaml'
    )
    
    rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_visualization'), 'launch'),
         '/agar_visualization_launch.py'])
      )
    
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_gazebo'), 'launch'),
         '/agar_gazebo_launch.py'])
      )
    
    description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_description'), 'launch'),
         '/agar_description_launch.py'])
      )

    return LaunchDescription([
        # Node(
        #     package='agar_bringup',
        #     namespace='/',
        #     executable='test_node',
        #     name='test_node',
        #     parameters=[config]
        # ),
        # rviz_launch,
        description_launch,
        gazebo_launch,

        # Node(
        #     package='agar_bringup',
        #     namespace='/',
        #     executable='agar_rosbag',
        #     name='agar_rosbag'
        # ),
    ])
    