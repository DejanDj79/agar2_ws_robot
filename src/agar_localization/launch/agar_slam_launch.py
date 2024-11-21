import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_slam_toolbox = FindPackageShare(package='slam_toolbox').find('slam_toolbox')   

    config = os.path.join(
      get_package_share_directory('agar_localization'),
      'config',
      'slamtoolbox_config.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
      description='Use simulation (Gazebo) clock if true')

    # slam_toolbox = Node(
    #     name='slamtoolbox',
    #     package='slam_toolbox', 
    #     executable='online_async_launch.py',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'slam_params_file': config,
    #     }],
    # )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        # launch_arguments={'slam_params_file': config}.items(),
        launch_arguments={'use_sime_time': use_sim_time, 'slam_params_file': config}.items(),
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_toolbox)
    return ld
