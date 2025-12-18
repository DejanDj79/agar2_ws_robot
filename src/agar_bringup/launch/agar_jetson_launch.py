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
    
    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('agar_navigation'), 'launch'),
         '/agar_navigation_launch.py'])
      )

    waypoint_follower = Node(
      package='agar_scripts', 
      executable='waypoint_follower',
      output='screen'
    )

    toknav = Node(
      package='agar_scripts', 
      executable='toknav_gps',
      output='screen',
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
      output='screen',
      respawn=True
    )

    object_detection = Node(
      package='agar_scripts', 
      executable='object_detection',
      output='screen',
      respawn=True
    )

    camera_mount_position = Node(
      package='agar_scripts', 
      executable='camera_mount_position',
      output='screen',
      respawn=True
    )

    collision_checker = Node(
      package='agar_scripts', 
      executable='collision_checker',
      output='screen',
      respawn=True
    )

    battery_status = Node(
      package='agar_scripts', 
      executable='battery_status',
      output='screen',
      respawn=True
    )

    jetson_online = Node(
      package='agar_scripts', 
      executable='jetson_status_pub',
      output='screen',
      respawn=True
    )

    zedcamera_front_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('zed_wrapper'), 'launch'),
         '/zed_camera.launch.py']),
          launch_arguments={
              'camera_model': 'zedx',
              'camera_name': 'zedx',
              'serial_number': '45931755',
          }.items()
      )
    
    zedcamera_rear_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('zed_wrapper'), 'launch'),
         '/zed_camera.launch.py']),
          launch_arguments={
              'camera_model': 'zedxm',
              'camera_name': 'zedxm',
              'serial_number': '55327043',
              'node_log_type': 'screen',
          }.items()
      )
    
    return LaunchDescription([
        toknav,
        camera_mount_position,
        description_launch,
        localization_launch,
        navigation_launch,
        waypoint_follower,
        object_detection,
        # collision_checker,
        # battery_status,
        jetson_online,

        waypoint_service,
        waypoint_files_service,
        get_mission_editor_service,
        set_mission_editor_service,
        position_data_service,

        zedcamera_front_launch,
        # # zedcamera_rear_launch,
    ])
    
