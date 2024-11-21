import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_config = os.path.join(
        get_package_share_directory('agar_bringup'),
        'config',
        'sim_time.yaml'
    )

    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    declare_qos = DeclareLaunchArgument(
        'qos', default_value='2',
        description='QoS used for input sensor topics')
    
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.')
    
    parameters={
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',
        'approx_sync': True,
        'Grid/RangeMax': '30.0',
        'Grid/RangeMin': '0.8',
        'Grid/MinGroundHeight': '0.0',
        'Grid/MaxGroundHeight': '2.0',
        'Grid/FromDepth': False,

        # 'Kp/MaxDepth': '4.0',
        # 'Kp/MinDepth': '0.0',

        # 'wait_for_transform': 0.2,
        # RTAB-Map's internal parameters are strings:
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/CreateOccupancyGrid': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Mem/LaserScanNormalK': '20',
        'Icp/VoxelSize': '0.05',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/RangeMax': '30.0',
        'Icp/RangeMin': '0.1',

        # 'Optimizer/Slam2D': 'true',
    }

    remappings=[
          ('odom', '/odometry/filtered'),
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw'),
          ('scan', '/lidar_scan'),
          ('scan_cloud', '/velodyne_points'),
    ]

    rtabmap_slam =  Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[use_sim_time_config, parameters],
        remappings=remappings,
        arguments=['-d'])
    
    rtabmap_slam_mem = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[use_sim_time_config, parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings)

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[use_sim_time_config, parameters],
        remappings=remappings)

    ld = LaunchDescription()
    ld.add_action(declare_qos)
    ld.add_action(declare_localization)

    ld.add_action(rtabmap_slam)
    # ld.add_action(rtabmap_slam_mem)
    # ld.add_action(rtabmap_viz)
    
    return ld