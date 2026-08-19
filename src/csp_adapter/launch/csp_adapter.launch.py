import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csp_adapter',
            executable='csp_adapter',
            name='csp_adapter',
            output='screen',
            arguments=[
                '--plan-path', os.environ.get(
                    'CSP_PLAN_PATH', 'results/example_run/flight_plan.json'),
                '--offset-x', os.environ.get('CSP_OFFSET_X', '0.0'),
                '--offset-y', os.environ.get('CSP_OFFSET_Y', '0.0'),
                '--offset-z', os.environ.get('CSP_OFFSET_Z', '0.0'),
                '--yaw-deg', os.environ.get('CSP_YAW_DEG', '0.0'),
                '--interval', os.environ.get('CSP_INTERVAL', '0.5'),
                '--land-delay', os.environ.get('CSP_LAND_DELAY', '5.0'),
            ],
        ),
    ])
