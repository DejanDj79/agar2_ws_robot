from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    sensor_frame = LaunchConfiguration('sensor_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_tf = LaunchConfiguration('publish_tf')
    tf_time_offset_sec = LaunchConfiguration('tf_time_offset_sec')
    preserve_odom_continuity = LaunchConfiguration(
        'preserve_odom_continuity'
    )
    reset_translation_threshold_m = LaunchConfiguration(
        'reset_translation_threshold_m'
    )
    reset_yaw_threshold_deg = LaunchConfiguration(
        'reset_yaw_threshold_deg'
    )
    sensor_x_m = LaunchConfiguration('sensor_x_m')
    sensor_y_m = LaunchConfiguration('sensor_y_m')
    sensor_z_m = LaunchConfiguration('sensor_z_m')
    sensor_roll_rad = LaunchConfiguration('sensor_roll_rad')
    sensor_pitch_rad = LaunchConfiguration('sensor_pitch_rad')
    sensor_yaw_rad = LaunchConfiguration('sensor_yaw_rad')

    odometry_adapter = Node(
        package='agar_slam',
        executable='zed_odom_to_base',
        name='zed_odom_to_base',
        output='screen',
        respawn=True,
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'odom_frame': odom_frame,
            'sensor_frame': sensor_frame,
            'base_frame': base_frame,
            'publish_tf': ParameterValue(publish_tf, value_type=bool),
            'tf_time_offset_sec': ParameterValue(
                tf_time_offset_sec,
                value_type=float,
            ),
            'preserve_odom_continuity': ParameterValue(
                preserve_odom_continuity,
                value_type=bool,
            ),
            'reset_translation_threshold_m': ParameterValue(
                reset_translation_threshold_m,
                value_type=float,
            ),
            'reset_yaw_threshold_deg': ParameterValue(
                reset_yaw_threshold_deg,
                value_type=float,
            ),
            'sensor_x_m': ParameterValue(sensor_x_m, value_type=float),
            'sensor_y_m': ParameterValue(sensor_y_m, value_type=float),
            'sensor_z_m': ParameterValue(sensor_z_m, value_type=float),
            'sensor_roll_rad': ParameterValue(
                sensor_roll_rad,
                value_type=float,
            ),
            'sensor_pitch_rad': ParameterValue(
                sensor_pitch_rad,
                value_type=float,
            ),
            'sensor_yaw_rad': ParameterValue(
                sensor_yaw_rad,
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/zedx/zed_node/odom',
            description='Raw ZED camera-frame odometry topic.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/odometry/filtered',
            description='Corrected base-frame odometry topic.',
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument(
            'sensor_frame',
            default_value='zedx_camera_link',
        ),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            choices=['true', 'false'],
            description='Publish the corrected odom -> base transform.',
        ),
        DeclareLaunchArgument(
            'tf_time_offset_sec',
            default_value='0.0',
            description='Optional future timestamp offset for odom TF.',
        ),
        DeclareLaunchArgument(
            'preserve_odom_continuity',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Keep odom continuous across abrupt ZED tracking resets.'
            ),
        ),
        DeclareLaunchArgument(
            'reset_translation_threshold_m',
            default_value='0.5',
            description=(
                'Single-message translation step treated as a ZED reset.'
            ),
        ),
        DeclareLaunchArgument(
            'reset_yaw_threshold_deg',
            default_value='10.0',
            description='Single-message rotation treated as a ZED reset.',
        ),
        DeclareLaunchArgument('sensor_x_m', default_value='0.6'),
        DeclareLaunchArgument('sensor_y_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_z_m', default_value='1.6'),
        DeclareLaunchArgument('sensor_roll_rad', default_value='0.0'),
        DeclareLaunchArgument('sensor_pitch_rad', default_value='0.0'),
        DeclareLaunchArgument('sensor_yaw_rad', default_value='0.0'),
        LogInfo(
            msg=[
                'Using corrected ZED base odometry: ',
                input_topic,
                ' -> ',
                output_topic,
            ],
        ),
        odometry_adapter,
    ])
