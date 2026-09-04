"""gz_sensor_interface launch — bring up the simulation-interaction layer.

Launches the Gazebo sensor bridge + coordinate conversions for the swan_gamma
drone (topics/params read from src/navigation/config/gz_sensor_interface.yaml):
  - lidar_sensor : transform left/right/horizontal LiDARs into base_link and
                   publish four body-frame outputs (merged/left/right/top)
  - imu_bridge   : /livox/imu_raw -> /livox/imu (monotonic stamps)
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


def _navigation_config_path() -> Path:
    """Return the workspace navigation sensor configuration path."""
    override = os.environ.get("YUNGU_SIM_CONFIG")
    if override:
        candidate = Path(override).resolve().parent / "gz_sensor_interface.yaml"
        if candidate.is_file():
            return candidate
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "navigation" / "config" / "gz_sensor_interface.yaml"
        if candidate.is_file():
            return candidate
    return Path("src/navigation/config/gz_sensor_interface.yaml")


def generate_launch_description():
    config_path = str(_navigation_config_path())
    model = _cfg(config_path, 'model', 'swan_gamma_v2')

    raw_left = _cfg(config_path, 'lidar_sensor.input_left',
                    f'/{model}/scan_left/points')
    raw_right = _cfg(config_path, 'lidar_sensor.input_right',
                     f'/{model}/scan_right/points')
    raw_h = _cfg(config_path, 'lidar_sensor.input_horizontal',
                 f'/{model}/scan_horizontal/points')
    merged = _cfg(config_path, 'lidar_sensor.output_merged',
                  f'/{model}/scan/points_fused')
    body_left = _cfg(config_path, 'lidar_sensor.output_left',
                     f'/{model}/scan_left/points_body')
    body_right = _cfg(config_path, 'lidar_sensor.output_right',
                      f'/{model}/scan_right/points_body')
    body_top = _cfg(config_path, 'lidar_sensor.output_top',
                    f'/{model}/scan_horizontal/points_body')


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model,
                              description='Gazebo model prefix for sensor topics'),
        Node(
            package='gz_sensor_interface', executable='lidar_sensor', name='lidar_sensor',
            output='screen',
            parameters=[{
                'input_left': raw_left,
                'input_right': raw_right,
                'input_horizontal': raw_h,
                'output_merged': merged,
                'output_left': body_left,
                'output_right': body_right,
                'output_top': body_top,
                'time_sync_tol': _cfg(config_path, 'lidar_sensor.time_sync_tol', 0.05),
                'left.t': _cfg(config_path, 'lidar_sensor.left.t', [0.0, 0.40, 0.05]),
                'left.roll': _cfg(config_path, 'lidar_sensor.left.roll', -0.6),
                'right.t': _cfg(config_path, 'lidar_sensor.right.t', [0.0, -0.40, 0.05]),
                'right.roll': _cfg(config_path, 'lidar_sensor.right.roll', 0.6),
                'horizontal.t': _cfg(config_path, 'lidar_sensor.horizontal.t',
                                     [0.0, 0.0, 0.16]),
                'horizontal.roll': _cfg(config_path, 'lidar_sensor.horizontal.roll', 0.0),
            }],
        ),
        Node(
            package='gz_sensor_interface', executable='imu_bridge', name='imu_bridge',
            output='screen',
            parameters=[{
                'input_topic': _cfg(config_path, 'imu_bridge.input_topic', '/livox/imu_raw'),
                'output_topic': _cfg(config_path, 'imu_bridge.output_topic', '/livox/imu'),
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
        # Note: super_lidar's in_cloud defaults to the topic matching cloud_source
        # (one of lidar_sensor's four base_link outputs). The `in_cloud` config
        # value (if set) always overrides.
    ])
