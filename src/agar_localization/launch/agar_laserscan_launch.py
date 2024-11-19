from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    velodyne_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="velodyne_to_laser",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', '1', 'velodyne', 'lidar_laserscan'],
    )

    pointcloud_to_laserscan = Node(
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/velodyne_points'),
                    ('scan', '/lidar_scan')],
        parameters=[{
            'target_frame': 'lidar_laserscan',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.1415,  # -M_PI/2
            'angle_max': 3.1415,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.2, # 0.3333,
            'range_min': 0.45,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sime_time': False
        }],
    )

    camera_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_laser",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', '1', 'depth_camera', 'camera_laserscan'],
    )

    depthimage_to_laserscan = Node(
        name='depthimage_to_laserscan',
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/camera_scan')
        ],
        parameters=[{
            'output_frame': 'camera_laserscan',
            'scan_height': 80,
            'scan_time': 0.2,
            'range_min': 0.45,
            'range_max': 30.0,
            'use_sime_time': True,
        }],
    )
    
    ld = LaunchDescription()
    ld.add_action(velodyne_to_laser)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(camera_to_laser)
    ld.add_action(depthimage_to_laserscan)
    
    return ld