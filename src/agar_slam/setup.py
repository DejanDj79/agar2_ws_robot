from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agar_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikolica',
    maintainer_email='nikola.velcev@coming.rs',
    description='AgAR SLAM launch files, configuration, and map tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_map_manager = agar_slam.slam_map_manager:manager_main',
            'save_slam_map = agar_slam.slam_map_manager:save_cli_main',
            'save_slam_posegraph = agar_slam.slam_map_manager:save_posegraph_cli_main',
            'set_active_slam_map = agar_slam.slam_map_manager:set_active_cli_main',
            'list_slam_maps = agar_slam.slam_map_manager:list_cli_main',
            'offline_localization_evaluator = agar_slam.offline_localization_evaluator:main',
            'offline_jump_diagnostics = agar_slam.offline_jump_diagnostics:main',
            'offline_odometry_tf = agar_slam.offline_odometry_tf:main',
            'slam_health_monitor = agar_slam.slam_health_monitor:main',
            'velodyne_ring_laserscan = agar_slam.velodyne_ring_laserscan:main',
            'zed_odom_to_base = agar_slam.zed_odom_to_base:main',
        ],
    },
)
