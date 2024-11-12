import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('agar_visualization'), 'config', 'agar_simulator.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    start_rviz = Node(
        package='rviz2',
        namespace='/',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_rviz)

    return ld
    