import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agar_gazebo'
submodules = 'agar_common'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikola Velcev',
    maintainer_email='nikola.velcev@coming.rs',
    description='AgAR Gazebo package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
