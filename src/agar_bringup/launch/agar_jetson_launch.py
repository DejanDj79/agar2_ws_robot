import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_localization'), 'launch'),
         '/agar_localization_launch.py'])
      )

    waypoint_follower = Node(
      package='agar_scripts', 
      executable='waypoint_follower',
      output='screen'
    )

    toknav = Node(
      package='agar_scripts', 
      executable='toknav_gps',
      output='screen'
    )

    datum_correction = Node(
      package='agar_scripts', 
      executable='datum_correction',
      output='screen'
    )

    return LaunchDescription([
        localization_launch,
        waypoint_follower,
        # toknav,
        datum_correction,
    ])
    