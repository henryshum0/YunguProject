import os
from glob import glob

from setuptools import setup

package_name = 'csp_adapter'

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
    description='flight_plan.json -> /waypoint_pose adapter (coverage-search-planner integration)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'csp_adapter = csp_adapter.csp_adapter:main',
        ],
    },
)
