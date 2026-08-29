import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_coord_bridge',
            executable='map_coord_bridge',
            name='map_coord_bridge',
            output='screen',
            arguments=[
                '--offset-x', os.environ.get('MCB_OFFSET_X', '0.0'),
                '--offset-y', os.environ.get('MCB_OFFSET_Y', '0.0'),
                '--offset-z', os.environ.get('MCB_OFFSET_Z', '0.0'),
                '--yaw-deg', os.environ.get('MCB_YAW_DEG', '0.0'),
                '--map-in-topic', os.environ.get(
                    'MCB_MAP_IN', '/map_pose_in'),
                '--world-out-topic', os.environ.get(
                    'MCB_WORLD_OUT', '/waypoint_pose'),
                '--world-in-topic', os.environ.get(
                    'MCB_WORLD_IN', '/lidar_slam/odom'),
                '--map-out-topic', os.environ.get(
                    'MCB_MAP_OUT', '/map/vehicle_odom'),
            ],
        ),
    ])
