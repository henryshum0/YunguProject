from setuptools import setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/offboard_position.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PX4 offboard control via position setpoints',
    license='BSD',
    entry_points={
        'console_scripts': [
            'position_controller = offboard_control.position_controller:main',
        ],
    },
)
