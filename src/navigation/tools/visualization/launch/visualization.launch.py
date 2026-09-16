"""visualization.launch — bring up the visualization layer.

Launches (topics/params read from src/simulation/config/visualization.yaml):
  - visual_tf         : TF tree anchored at the drone launch-origin world frame
  - gt_path           : Gazebo truth -> /gt_path in the launch-origin world
  - rviz2 (optional)  : freelook 3D window

The visualization world frame is anchored at the drone launch position. Gazebo
truth odom is shifted by the spawn offset so it aligns with EGO in RViz.
"""
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def _airframe_spawn(project_root, model):
    """Read PX4_GZ_MODEL_POSE from the airframe file; fall back to env."""
    import re
    spawn = os.environ.get('PX4_GZ_MODEL_POSE', '')
    if spawn:
        return spawn
    airframes = (project_root / 'VisionFlow-PX4' / 'ROMFS' / 'px4fmu_common'
                 / 'init.d-posix' / 'airframes')
    if not airframes.is_dir():
        return ''
    for f in sorted(airframes.glob(f'*_gz_{model}')):
        m = re.search(r'PX4_GZ_MODEL_POSE=.*"([-0-9.,]+)"',
                      f.read_text(encoding='utf-8', errors='ignore'))
        if m:
            return m.group(1)
    return ''


def _find_workspace_root() -> Path:
    """Locate the workspace containing the simulation configuration."""
    override = os.environ.get("YUNGU_SIM_CONFIG")
    if override:
        simulation_config = Path(override).resolve()
        if simulation_config.is_file():
            return simulation_config.parents[3]
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "simulation" / "config" / "visualization.yaml").is_file():
            return parent
    return Path(__file__).resolve().parents[4]


def generate_launch_description():
    project_root = _find_workspace_root()
    config_path = str(project_root / 'src' / 'simulation' / 'config' / 'visualization.yaml')

    # Spawn offset (drone launch origin in gz) from the airframe; overridable.
    spawn = _airframe_spawn(project_root, 'swan_gamma_v2')
    sp_x, sp_y, sp_z = '0.0', '0.0', '0.0'
    if spawn:
        parts = spawn.split(',')
        if len(parts) >= 3:
            sp_x, sp_y, sp_z = parts[0], parts[1], parts[2]
    sp_x = _cfg(config_path, 'gt_path.spawn_offset_x', sp_x)
    sp_y = _cfg(config_path, 'gt_path.spawn_offset_y', sp_y)
    sp_z = _cfg(config_path, 'gt_path.spawn_offset_z', sp_z)

    world_frame = _cfg(config_path, 'frames.world', 'world')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value=str(
            _cfg(config_path, 'rviz.enabled', True)).lower(),
            description='Launch the RViz windows'),
        DeclareLaunchArgument('spawn_offset_x', default_value=str(sp_x),
                              description='Drone spawn X (PX4 ENU origin in gz)'),
        DeclareLaunchArgument('spawn_offset_y', default_value=str(sp_y),
                              description='Drone spawn Y (PX4 ENU origin in gz)'),
        DeclareLaunchArgument('spawn_offset_z', default_value=str(sp_z),
                              description='Drone spawn Z (PX4 ENU origin in gz)'),

        Node(
            package='visualization', executable='visual_tf', name='visual_tf',
            output='screen',
            parameters=[{
                'odom_topic': _cfg(config_path, 'visual_tf.odom_topic', '/gz/odom_super'),
                'world_frame': world_frame,
                'body_frame': _cfg(config_path, 'frames.body', 'body'),
                'base_frame': _cfg(config_path, 'frames.base_link', 'base_link'),
                'lidar_frame': _cfg(config_path, 'frames.lidar_link', 'lidar_link'),
                'lidar_offset_z': float(_cfg(config_path, 'frames.lidar_offset_z', 0.16)),
            }],
        ),
        Node(
            package='visualization', executable='gt_path.py', name='gt_path',
            output='screen',
            parameters=[{
                'input_topic': _cfg(config_path, 'gt_path.input_topic', '/gz/ground_truth/odom'),
                'output_topic': _cfg(config_path, 'gt_path.output_topic', '/gt_path'),
                'world_frame': _cfg(config_path, 'gt_path.world_frame', world_frame),
                'spawn_offset_x': LaunchConfiguration('spawn_offset_x'),
                'spawn_offset_y': LaunchConfiguration('spawn_offset_y'),
                'spawn_offset_z': LaunchConfiguration('spawn_offset_z'),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_freelook', output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('visualization'), 'rviz',
                _cfg(config_path, 'rviz.freelook_config', 'freelook.rviz')])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
