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
    
    rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_visualization'), 'launch'),
         '/agar_visualization_launch.py'])
      )
    
    mapviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_visualization'), 'launch'),
         '/agar_mapviz_launch.py'])
      )
    
    localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_localization'), 'launch'),
         '/agar_localization_launch.py'])
      )
    
    # robot_steering_node = Node(
    #   package='rqt_robot_steering', 
    #   executable='rqt_robot_steering',
    #   output='screen'
    # )

    fake_sensors = Node(
      package='agar_scripts', 
      executable='fake_sensors',
      output='screen'
    )

    waypoint_follower = Node(
      package='agar_scripts', 
      executable='waypoint_follower',
      output='screen'
    )

    datum_correction = Node(
      package='agar_scripts', 
      executable='datum_correction',
      output='screen'
    )

    return LaunchDescription([
        description_launch,
        localization_launch,
        rviz_launch,
        mapviz_launch,
        fake_sensors,
        waypoint_follower,
        datum_correction
    ])
    