import os
from glob import glob

from setuptools import setup

package_name = 'map_coord_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windshape',
    maintainer_email='windshape@example.com',
    description='planner-local ENU <-> world ENU coordinate bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_coord_bridge = map_coord_bridge.map_coord_bridge:main',
        ],
    },
)
