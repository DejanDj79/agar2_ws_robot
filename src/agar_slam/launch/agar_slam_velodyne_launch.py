import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_slam_toolbox = FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    default_config = os.path.join(
        get_package_share_directory('agar_slam'),
        'config',
        'slamtoolbox_velodyne_config.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    declare_slam_params_file = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=default_config,
        description='Full path to the slam_toolbox parameter file.',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file,
        slam_toolbox,
    ])
