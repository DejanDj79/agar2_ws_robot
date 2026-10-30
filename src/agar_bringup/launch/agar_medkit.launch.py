#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include_launch(package_name, launch_file, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            'launch',
            launch_file,
        )
    )
    kwargs = {}
    if launch_arguments:
        kwargs['launch_arguments'] = launch_arguments.items()
    return IncludeLaunchDescription(source, **kwargs)


def generate_launch_description():
    gateway_share = get_package_share_directory('ros2_medkit_gateway')
    bringup_share = get_package_share_directory('agar_bringup')

    gateway_defaults = os.path.join(
        gateway_share,
        'config',
        'gateway_params.yaml',
    )
    gateway_overrides = os.path.join(
        bringup_share,
        'config',
        'ros2_medkit_gateway.yaml',
    )
    fault_manager_params = os.path.join(
        bringup_share,
        'config',
        'ros2_medkit_fault_manager.yaml',
    )
    bridge_params = os.path.join(
        bringup_share,
        'config',
        'ros2_medkit_bridge.yaml',
    )

    gateway = Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[gateway_defaults, gateway_overrides],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    fault_manager = include_launch(
        'ros2_medkit_fault_manager',
        'fault_manager.launch.py',
        {'params_file': fault_manager_params},
    )
    diagnostic_bridge = Node(
        package='agar_bringup',
        executable='agar_medkit_diagnostic_bridge',
        name='agar_medkit_diagnostic_bridge',
        output='screen',
        parameters=[bridge_params],
    )

    return LaunchDescription([
        gateway,
        fault_manager,
        diagnostic_bridge,
    ])
