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
            'toknav_odom = agar_scripts.toknav_odom:main',
        ],
    },
)
