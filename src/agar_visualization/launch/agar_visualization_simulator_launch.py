import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('agar_visualization'), 'config', 'agar_simulator.rviz')

    use_sim_time_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'sim_time.yaml'
    )
    
    start_rviz = Node(
        package='rviz2',
        namespace='/',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config],
        parameters=[use_sim_time_config]
    )

    ld = LaunchDescription()
    ld.add_action(start_rviz)

    return ld
    