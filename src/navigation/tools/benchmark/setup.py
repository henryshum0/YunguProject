import os
from glob import glob

from setuptools import setup

package_name = 'benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='windshape',
    maintainer_email='windshape@example.com',
    description='Gazebo benchmark world generator (flat map with random gate/pillar obstacles)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'generate_map = benchmark.generate_map:main',
            'waypoint_populator = benchmark.waypoint_populator:main',
        ],
    },
)
