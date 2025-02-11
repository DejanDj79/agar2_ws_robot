import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []

    use_sim_time_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'sim_time.yaml'
    )
    
    pkg_agar_description = FindPackageShare(package='agar_description').find('agar_description')
    urdf_path = os.path.join(pkg_agar_description, 'urdf', 'agar_low_poly.xacro')

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        # package="joint_state_publisher_gui",
        # executable="joint_state_publisher_gui",
        parameters=[
            use_sim_time_config,
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            use_sim_time_config,
            {'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_path)]), value_type=str
            )}
        ],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)