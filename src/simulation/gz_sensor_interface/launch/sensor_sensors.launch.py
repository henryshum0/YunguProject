"""gz_sensor_interface launch — bring up the simulation-interaction layer.

Launches the Gazebo sensor bridge + coordinate conversions for the swan_gamma
drone (topics/params read from src/simulation/config/gz_sensor_interface.yaml):
  - lidar_sensor : transform the horizontal LiDAR into base_link
  - truth_odom   : /odom -> /gz/ground_truth/odom
"""
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cfg(path, key, default=None):
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        node = data
        for part in key.split('.'):
            node = node[part]
        return node
    except Exception:
        return default


def _simulation_config_path() -> Path:
    """Return the workspace simulation sensor configuration path."""
    override = os.environ.get("YUNGU_SIM_CONFIG")
    if override:
        candidate = Path(override).resolve().parent / "gz_sensor_interface.yaml"
        if candidate.is_file():
            return candidate
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "simulation" / "config" / "gz_sensor_interface.yaml"
        if candidate.is_file():
            return candidate
    return Path("src/simulation/config/gz_sensor_interface.yaml")


def generate_launch_description():
    config_path = str(_simulation_config_path())
    model = _cfg(config_path, 'model', 'swan_gamma_v2')

    raw_h = _cfg(config_path, 'lidar_sensor.input_horizontal',
                 f'/{model}/scan_horizontal/points')
    body_h = _cfg(config_path, 'lidar_sensor.output_horizontal',
                  f'/{model}/scan_horizontal/points_body')


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model,
                              description='Gazebo model prefix for sensor topics'),
        Node(
            package='gz_sensor_interface', executable='lidar_sensor', name='lidar_sensor',
            output='screen',
            parameters=[{
                'input_horizontal': raw_h,
                'output_horizontal': body_h,
                'horizontal.t': _cfg(config_path, 'lidar_sensor.horizontal.t',
                                     [0.0, 0.0, 0.16]),
                'horizontal.roll': _cfg(config_path, 'lidar_sensor.horizontal.roll', 0.0),
            }],
        ),
        Node(
            package='gz_sensor_interface', executable='truth_odom', name='truth_odom',
            output='screen',
            parameters=[{
                'input_topic': _cfg(config_path, 'truth_odom.input_topic', '/odom'),
                'output_topic': _cfg(config_path, 'truth_odom.output_topic',
                                     '/gz/ground_truth/odom'),
            }],
        ),
        Node(
            package='gz_sensor_interface', executable='super_lidar', name='super_lidar',
            output='screen',
            parameters=[{
                'in_cloud': _cfg(config_path, 'super_lidar.in_cloud','/swan_gamma_v2/scan_horizontal/points_body'), 
                'in_odom': _cfg(config_path, 'super_lidar.in_odom',
                                '/fmu/out/vehicle_odometry'),
                'out_cloud': _cfg(config_path, 'super_lidar.out_cloud',
                                  '/gz/point_cloud_super'),
                'out_odom': _cfg(config_path, 'super_lidar.out_odom', '/gz/odom_super'),
            }],
        ),
        # The horizontal body-frame cloud is the default SUPER input. The
        # configured in_cloud value (if set) always overrides it.
    ])
