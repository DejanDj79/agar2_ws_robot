import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_gazebo'), 'launch'),
         '/agar_gazebo_launch.py'])
      )
    
    description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_description'), 'launch'),
         '/agar_description_simulator_launch.py'])
      )
    
    rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_visualization'), 'launch'),
         '/agar_visualization_simulator_launch.py'])
      )
    
    localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_localization'), 'launch'),
         '/agar_localization_simulator_launch.py'])
      )
    
    mapviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_visualization'), 'launch'),
         '/agar_mapviz_launch.py'])
      )
    
    robot_steering_node = Node(
      package='rqt_robot_steering', 
      executable='rqt_robot_steering',
      output='screen'
    )

    waypoint_follower = Node(
      package='agar_scripts', 
      executable='waypoint_follower',
      output='screen'
    )

    # datum_correction = Node(
    #   package='agar_scripts', 
    #   executable='datum_correction',
    #   output='screen'
    # )

    waypoint_service = Node(
      package='agar_scripts', 
      executable='waypoint_service',
      output='screen'
    )

    waypoint_files_service = Node(
      package='agar_scripts', 
      executable='waypoint_files_service',
      output='screen'
    )

    get_mission_editor_service = Node(
      package='agar_scripts', 
      executable='get_mission_editor_service',
      output='screen'
    )

    set_mission_editor_service = Node(
      package='agar_scripts', 
      executable='set_mission_editor_service',
      output='screen'
    )
     
    position_data_service = Node(
      package='agar_scripts', 
      executable='position_data_service',
      output='screen'
    )

    return LaunchDescription([
        description_launch,
        localization_launch,
        # gazebo_launch,
        rviz_launch,
        # mapviz_launch,
        # robot_steering_node,

        # waypoint_follower,
        # datum_correction,

        waypoint_service,
        waypoint_files_service,
        get_mission_editor_service,
        set_mission_editor_service,
        position_data_service,


        # Node(
        #     package='agar_bringup',
        #     namespace='/',
        #     executable='agar_rosbag',
        #     name='agar_rosbag'
        # ),
    ])
    