import os
import re

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


def safe_filename(value):
    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(value).strip())
    return value.strip('._') or 'offline_mapping'


def required_bag(path):
    path = os.path.abspath(os.path.expanduser(path))
    metadata_path = os.path.join(path, 'metadata.yaml')
    if not os.path.isdir(path) or not os.path.isfile(metadata_path):
        raise RuntimeError(
            'ROS bag directory does not contain metadata.yaml: {}'.format(path)
        )
    return path


def launch_setup(context):
    bag_path = required_bag(LaunchConfiguration('bag').perform(context))
    scan_source = LaunchConfiguration('scan_source').perform(context)
    odometry_source = LaunchConfiguration('odometry_source').perform(context)
    slam_profile = LaunchConfiguration('slam_profile').perform(context)
    output_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('output_dir').perform(context)
    ))
    run_name = safe_filename(LaunchConfiguration('run_name').perform(context))
    play_rate = float(LaunchConfiguration('play_rate').perform(context))
    playback_delay = float(
        LaunchConfiguration('playback_delay_sec').perform(context)
    )
    bag_discovery_delay = float(
        LaunchConfiguration('bag_discovery_delay_sec').perform(context)
    )
    map_save_delay = float(
        LaunchConfiguration('map_save_delay_sec').perform(context)
    )
    sample_rate_hz = float(
        LaunchConfiguration('sample_rate_hz').perform(context)
    )
    translation_jump_m = float(
        LaunchConfiguration('translation_jump_m').perform(context)
    )
    yaw_jump_deg = float(
        LaunchConfiguration('yaw_jump_deg').perform(context)
    )

    scan_profiles = {
        'single_ring': {
            'topic': '/scan',
            'rings': None,
        },
        'multi_ring': {
            'topic': '/scan_multiring',
            'rings': None,
        },
        'three_ring': {
            'topic': '/scan_ring_789',
            'rings': [7, 8, 9],
        },
        'five_ring': {
            'topic': '/scan_ring_678910',
            'rings': [6, 7, 8, 9, 10],
        },
    }
    odometry_profiles = {
        'recorded_filtered': {
            'topic': '/odometry/filtered',
            'convert_zed': False,
        },
        'zed_base_corrected': {
            'topic': '/zedx/odom_base',
            'convert_zed': True,
        },
    }
    slam_profiles = {
        'current': {},
        'zed_constrained': {
            'do_loop_closing': False,
            'link_match_minimum_response_fine': 0.4,
            'correlation_search_space_dimension': 0.3,
            'correlation_search_space_smear_deviation': 0.03,
            'coarse_search_angle_offset': 0.0872665,
            'coarse_angle_resolution': 0.0174533,
            'use_response_expansion': False,
        },
    }

    if scan_source not in scan_profiles:
        raise RuntimeError(
            'Unsupported scan_source: {}'.format(scan_source)
        )
    if odometry_source not in odometry_profiles:
        raise RuntimeError(
            'Unsupported odometry_source: {}'.format(odometry_source)
        )
    if slam_profile not in slam_profiles:
        raise RuntimeError(
            'Unsupported slam_profile: {}'.format(slam_profile)
        )
    if play_rate <= 0.0:
        raise RuntimeError('play_rate must be greater than zero')
    if min(playback_delay, bag_discovery_delay, map_save_delay) < 0.0:
        raise RuntimeError('launch delays cannot be negative')
    if sample_rate_hz <= 0.0:
        raise RuntimeError('sample_rate_hz must be greater than zero')
    if translation_jump_m <= 0.0 or yaw_jump_deg <= 0.0:
        raise RuntimeError('jump thresholds must be greater than zero')

    os.makedirs(output_dir, exist_ok=True)

    pkg_slam = get_package_share_directory('agar_slam')
    slam_toolbox_config = os.path.join(
        pkg_slam,
        'config',
        'slamtoolbox_velodyne_config.yaml',
    )
    multiring_config = os.path.join(
        pkg_slam,
        'config',
        'velodyne_multiring_scan.yaml',
    )
    rosbag_qos_config = os.path.join(
        pkg_slam,
        'config',
        'offline_rosbag_qos.yaml',
    )

    scan_topic = scan_profiles[scan_source]['topic']
    odometry_topic = odometry_profiles[odometry_source]['topic']
    result_name = '{}_{}_{}_{}'.format(
        run_name,
        scan_source,
        odometry_source,
        slam_profile,
    )
    map_output_base = os.path.join(output_dir, result_name + '_map')

    actions = [
        LogInfo(msg='Offline mapping scan source: {}'.format(scan_source)),
        LogInfo(msg='Offline odometry source: {}'.format(odometry_source)),
        LogInfo(msg='Offline SLAM profile: {}'.format(slam_profile)),
        LogInfo(msg='Offline ROS bag: {}'.format(bag_path)),
        LogInfo(msg='Offline results: {}'.format(output_dir)),
    ]

    if odometry_profiles[odometry_source]['convert_zed']:
        actions.append(Node(
            package='agar_slam',
            executable='zed_odom_to_base',
            name='zed_odom_to_base',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/zedx/zed_node/odom',
                'output_topic': odometry_topic,
                'sensor_frame': 'zedx_camera_link',
                'base_frame': 'base_footprint',
                'sensor_x_m': 0.6,
                'sensor_y_m': 0.0,
                'sensor_z_m': 1.6,
                'sensor_roll_rad': 0.0,
                'sensor_pitch_rad': 0.0,
                'sensor_yaw_rad': 0.0,
            }],
        ))

    actions.append(Node(
        package='agar_slam',
        executable='offline_odometry_tf',
        name='offline_odometry_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': odometry_topic,
        }],
    ))

    if scan_source == 'multi_ring':
        actions.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='velodyne_multiring_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('scan', scan_topic),
            ],
            parameters=[
                multiring_config,
                {'use_sim_time': True},
            ],
        ))

    elif scan_source in ('three_ring', 'five_ring'):
        actions.append(Node(
            package='agar_slam',
            executable='velodyne_ring_laserscan',
            name='velodyne_ring_laserscan',
            output='screen',
            remappings=[
                ('velodyne_points', '/velodyne_points'),
                ('scan', scan_topic),
            ],
            parameters=[{
                'use_sim_time': True,
                'rings': scan_profiles[scan_source]['rings'],
                'resolution': 0.007,
                'range_min': 0.0,
                'range_max': 200.0,
            }],
        ))

    actions.append(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config,
            slam_profiles[slam_profile],
            {
                'use_sim_time': True,
                'scan_topic': scan_topic,
                'transform_publish_period': 0.05,
                'scan_queue_size': 20,
                'enable_interactive_mode': False,
            },
        ],
    ))

    actions.append(Node(
        package='agar_slam',
        executable='offline_localization_evaluator',
        name='offline_mapping_evaluator',
        output='screen',
        remappings=[
            ('/scan', scan_topic),
            ('/odometry/filtered', odometry_topic),
        ],
        parameters=[{
            'use_sim_time': True,
            'localizer': 'mapping_{}_{}_{}'.format(
                scan_source,
                odometry_source,
                slam_profile,
            ),
            'run_name': run_name,
            'output_dir': output_dir,
            'sample_rate_hz': sample_rate_hz,
            'translation_jump_m': translation_jump_m,
            'yaw_jump_deg': yaw_jump_deg,
        }],
    ))

    actions.append(Node(
        package='agar_slam',
        executable='offline_jump_diagnostics',
        name='offline_jump_diagnostics',
        output='screen',
        remappings=[
            ('/scan', scan_topic),
            ('/odometry/filtered', odometry_topic),
        ],
        parameters=[{
            'use_sim_time': True,
            'localizer': 'mapping_{}_{}_{}'.format(
                scan_source,
                odometry_source,
                slam_profile,
            ),
            'run_name': run_name,
            'output_dir': output_dir,
            'sample_rate_hz': sample_rate_hz,
            'translation_jump_m': translation_jump_m,
            'yaw_jump_deg': yaw_jump_deg,
        }],
    ))

    bag_topics = [
        '/zedx/zed_node/odom',
        '/odometry/filtered',
        '/cmd_vel',
        '/imu/data',
        '/zedx/zed_node/imu/data',
        '/gps/fix',
        '/tf_static',
        '/scan' if scan_source == 'single_ring' else '/velodyne_points',
    ]
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
            '--delay',
            str(bag_discovery_delay),
            '--wait-for-all-acked',
            '10000',
            '--topics',
            *bag_topics,
        ],
        output='screen',
    )

    map_saver = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            map_output_base,
            '--ros-args',
            '-p',
            'save_map_timeout:=10.0',
        ],
        output='screen',
    )

    actions.append(RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg='ROS bag playback finished; saving map.'),
                TimerAction(period=map_save_delay, actions=[map_saver]),
            ],
        )
    ))
    actions.append(RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver,
            on_exit=[
                LogInfo(msg='Offline map save finished.'),
                EmitEvent(event=Shutdown(reason='Offline mapping finished')),
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
            'scan_source',
            default_value='single_ring',
            choices=[
                'single_ring', 'multi_ring', 'three_ring', 'five_ring',
            ],
            description='LaserScan source used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'odometry_source',
            default_value='recorded_filtered',
            choices=['recorded_filtered', 'zed_base_corrected'],
            description=(
                'Odometry used for odom -> base_footprint during playback.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_profile',
            default_value='current',
            choices=['current', 'zed_constrained'],
            description='SLAM Toolbox parameter profile used during playback.',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='~/workspace/slam_results',
            description=(
                'Directory for maps, trajectory CSV, and summary JSON.'
            ),
        ),
        DeclareLaunchArgument(
            'run_name',
            default_value='multiring_turns_01',
            description='Name included in result filenames.',
        ),
        DeclareLaunchArgument(
            'play_rate',
            default_value='0.5',
            description='ROS bag playback rate.',
        ),
        DeclareLaunchArgument(
            'playback_delay_sec',
            default_value='5.0',
            description='Wall-clock delay before ROS bag playback starts.',
        ),
        DeclareLaunchArgument(
            'bag_discovery_delay_sec',
            default_value='3.0',
            description=(
                'Delay after rosbag publishers are created for DDS discovery.'
            ),
        ),
        DeclareLaunchArgument(
            'map_save_delay_sec',
            default_value='30.0',
            description='Delay after playback before saving the final map.',
        ),
        DeclareLaunchArgument(
            'sample_rate_hz',
            default_value='10.0',
            description='Transform result sampling rate.',
        ),
        DeclareLaunchArgument(
            'translation_jump_m',
            default_value='0.25',
            description='Translation step recorded as a SLAM correction jump.',
        ),
        DeclareLaunchArgument(
            'yaw_jump_deg',
            default_value='5.0',
            description='Yaw step recorded as a SLAM correction jump.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
