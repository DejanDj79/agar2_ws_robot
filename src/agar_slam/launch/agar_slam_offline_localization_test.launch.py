import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RAW_BAG_TOPICS = [
    '/scan',
    '/zedx/zed_node/odom',
    '/zedx/zed_node/imu/data',
    '/gps/fix',
    '/cmd_vel',
    '/tf_static',
]


def required_path(path, description):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        raise RuntimeError('{} does not exist: {}'.format(description, path))
    return path


def required_bag(path):
    path = required_path(path, 'ROS bag')
    metadata_path = os.path.join(path, 'metadata.yaml')
    if not os.path.isdir(path) or not os.path.isfile(metadata_path):
        raise RuntimeError(
            'ROS bag directory does not contain metadata.yaml: {}'.format(path)
        )
    return path


def launch_setup(context):
    localizer = LaunchConfiguration('localizer').perform(context)
    bag_path = required_bag(LaunchConfiguration('bag').perform(context))
    map_path = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map').perform(context)
    ))
    posegraph_path = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('posegraph').perform(context)
    ))
    output_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('output_dir').perform(context)
    ))
    run_name = LaunchConfiguration('run_name').perform(context)
    play_rate = float(LaunchConfiguration('play_rate').perform(context))
    playback_delay = float(
        LaunchConfiguration('playback_delay_sec').perform(context)
    )
    initial_x = float(LaunchConfiguration('initial_x').perform(context))
    initial_y = float(LaunchConfiguration('initial_y').perform(context))
    initial_yaw = float(LaunchConfiguration('initial_yaw').perform(context))
    sample_rate_hz = float(
        LaunchConfiguration('sample_rate_hz').perform(context)
    )
    translation_jump_m = float(
        LaunchConfiguration('translation_jump_m').perform(context)
    )
    yaw_jump_deg = float(
        LaunchConfiguration('yaw_jump_deg').perform(context)
    )

    if play_rate <= 0.0:
        raise RuntimeError('play_rate must be greater than zero')
    if playback_delay < 0.0:
        raise RuntimeError('playback_delay_sec cannot be negative')
    if sample_rate_hz <= 0.0:
        raise RuntimeError('sample_rate_hz must be greater than zero')
    if translation_jump_m <= 0.0 or yaw_jump_deg <= 0.0:
        raise RuntimeError('jump thresholds must be greater than zero')
    if localizer == 'amcl':
        required_path(map_path, 'AMCL map YAML')
    if localizer == 'slam_toolbox':
        required_path(posegraph_path + '.posegraph', 'slam_toolbox posegraph')

    pkg_slam = get_package_share_directory('agar_slam')
    ekf_config = os.path.join(
        pkg_slam,
        'config',
        'localization_zed_odom_slam.yaml',
    )
    amcl_config = os.path.join(
        pkg_slam,
        'config',
        'amcl_velodyne_config.yaml',
    )
    slam_toolbox_config = os.path.join(
        pkg_slam,
        'config',
        'slamtoolbox_velodyne_localization_config.yaml',
    )
    rosbag_qos_config = os.path.join(
        pkg_slam,
        'config',
        'offline_rosbag_qos.yaml',
    )

    actions = [
        LogInfo(msg='Offline SLAM test mode: {}'.format(localizer)),
        LogInfo(msg='Offline ROS bag: {}'.format(bag_path)),
        LogInfo(msg='Offline results: {}'.format(output_dir)),
    ]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True},
        ],
    )
    actions.append(ekf_node)

    if localizer == 'amcl':
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                amcl_config,
                {
                    'use_sim_time': True,
                    'yaml_filename': map_path,
                },
            ],
        )
        amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config,
                {
                    'use_sim_time': True,
                    'set_initial_pose': True,
                    'always_reset_initial_pose': True,
                    'initial_pose.x': initial_x,
                    'initial_pose.y': initial_y,
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': initial_yaw,
                },
            ],
        )
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_offline_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        )
        actions.extend([map_server, amcl, lifecycle_manager])
    elif localizer == 'slam_toolbox':
        slam_toolbox = Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_toolbox_config,
                {
                    'use_sim_time': True,
                    'map_file_name': posegraph_path,
                    'map_start_pose': [initial_x, initial_y, initial_yaw],
                },
            ],
        )
        actions.append(slam_toolbox)

    evaluator = Node(
        package='agar_slam',
        executable='offline_localization_evaluator',
        name='offline_localization_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'localizer': localizer,
            'run_name': run_name,
            'output_dir': output_dir,
            'sample_rate_hz': sample_rate_hz,
            'translation_jump_m': translation_jump_m,
            'yaw_jump_deg': yaw_jump_deg,
        }],
    )
    actions.append(evaluator)

    bag_play = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock',
            '100.0',
            '--rate',
            str(play_rate),
            '--disable-keyboard-controls',
            '--qos-profile-overrides-path',
            rosbag_qos_config,
            '--topics',
            *RAW_BAG_TOPICS,
        ],
        output='screen',
    )
    actions.append(RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg='ROS bag playback finished.'),
                EmitEvent(event=Shutdown(reason='ROS bag playback finished')),
            ],
        )
    ))
    actions.append(TimerAction(
        period=playback_delay,
        actions=[bag_play],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag',
            description='Full path to the ROS2 bag directory.',
        ),
        DeclareLaunchArgument(
            'localizer',
            default_value='baseline',
            choices=['baseline', 'amcl', 'slam_toolbox'],
            description='Offline test mode.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/nvidia/workspace/maps/active_map.yaml',
            description='Map YAML used by AMCL.',
        ),
        DeclareLaunchArgument(
            'posegraph',
            default_value='/home/nvidia/workspace/maps/active_posegraph',
            description='Posegraph base path used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/nvidia/workspace/slam_results',
            description='Directory for trajectory CSV and summary JSON.',
        ),
        DeclareLaunchArgument(
            'run_name',
            default_value='outdoor_reference_01',
            description='Name included in result filenames.',
        ),
        DeclareLaunchArgument(
            'play_rate',
            default_value='1.0',
            description='ROS bag playback rate.',
        ),
        DeclareLaunchArgument(
            'playback_delay_sec',
            default_value='5.0',
            description='Wall-clock delay before ROS bag playback starts.',
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Initial robot X position in the saved map.',
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial robot Y position in the saved map.',
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Initial robot yaw in radians in the saved map.',
        ),
        DeclareLaunchArgument(
            'sample_rate_hz',
            default_value='10.0',
            description='Result sampling rate.',
        ),
        DeclareLaunchArgument(
            'translation_jump_m',
            default_value='0.25',
            description='Translation step recorded as a localization jump.',
        ),
        DeclareLaunchArgument(
            'yaw_jump_deg',
            default_value='5.0',
            description='Yaw step recorded as a localization jump.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
