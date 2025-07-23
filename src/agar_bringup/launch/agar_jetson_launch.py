import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_description'), 'launch'),
         '/agar_description_launch.py'])
      )
    
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
      # output='screen',
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
      # output='screen',
      respawn=True
    )

    return LaunchDescription([
        toknav,
        description_launch,
        localization_launch,
        waypoint_follower,
        # datum_correction,

        waypoint_service,
        waypoint_files_service,
        get_mission_editor_service,
        set_mission_editor_service,
        position_data_service,
    ])
    