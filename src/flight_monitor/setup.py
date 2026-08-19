import os
from glob import glob

from setuptools import setup

package_name = 'flight_monitor'

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
    description='Flight observability: fusion monitor + command-trajectory recorder/plotter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'monitor = flight_monitor.monitor:main',
            'cmd_record_node = flight_monitor.cmd_record_node:main',
            'plot_csv = flight_monitor.plot_csv:main',
        ],
    },
)
