from setuptools import find_packages, setup

package_name = 'agar_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikolica',
    maintainer_email='nikola.velcev@coming.rs',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_odometry = agar_scripts.simulator_odometry:main',
            'waypoint_follower = agar_scripts.waypoint_follower:main',
            'collision_checker = agar_scripts.collision_checker:main',
            'toknav_gps = agar_scripts.toknav:main',
            'datum_correction = agar_scripts.datum_correction:main',
            'rosbag_transforms = agar_scripts.rosbag_transforms:main',
            'fake_sensors = agar_scripts.fake_sensors:main',
            'fake_sensors_file = agar_scripts.fake_sensors_file:main',
            'waypoint_files_service = agar_scripts.waypoint_files_service:main',
            'waypoint_service = agar_scripts.waypoint_service:main',
            'get_mission_editor_service = agar_scripts.get_mission_editor_service:main',
            'set_mission_editor_service = agar_scripts.set_mission_editor_service:main',
            'position_data_service = agar_scripts.position_data_service:main',
            # 'notifications = agar_scripts.notifications:main',
            'object_detection = agar_scripts.object_detection:main',
            # 'telegram_commands = agar_scripts.telegram_commands:main',
            'camera_mount_position = agar_scripts.camera_mount_position:main',
            'battery_status = agar_scripts.battery_status:main',
            'jetson_status_pub = agar_scripts.jetson_status_pub:main',
        ],
    },
)
