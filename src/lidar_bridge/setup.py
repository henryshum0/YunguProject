import os
from glob import glob

from setuptools import setup

package_name = 'lidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windshape',
    maintainer_email='windshape@example.com',
    description='Dual-side LiDAR data-flow glue (time-field, merge, IMU relay, TF bridge)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'add_time_field = lidar_bridge.add_time_field:main',
            'lidar_merge = lidar_bridge.lidar_merge:main',
            'imu_relay = lidar_bridge.imu_relay:main',
            'tf_bridge = lidar_bridge.tf_bridge:main',
        ],
    },
)
