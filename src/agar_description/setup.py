import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agar_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.[stl]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikola Velcev',
    maintainer_email='nikola.velcev@coming.rs',
    description='AgAR model description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
