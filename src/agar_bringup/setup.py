import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agar_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikolica',
    maintainer_email='nikolica@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = agar_bringup.test_node:main',
            'agar_rosbag = agar_bringup.rosbag_record:main',
            'agar_medkit_diagnostic_bridge = agar_bringup.medkit_diagnostic_bridge:main',
        ],
    },
)
