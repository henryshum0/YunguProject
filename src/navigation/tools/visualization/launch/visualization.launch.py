"""visualization.launch — bring up the visualization layer.

Launches (topics/params read from src/navigation/config/visualization.yaml):
  - visual_tf         : TF tree anchored at the drone launch-origin world frame
  - gt_path           : Gazebo truth -> /gt_path in the launch-origin world
  - fastlio_visual    : FAST-LIO cloud/odom republished in the visualization world
  - birdview_publisher: aerial top-down map overlay (optional)
  - rviz2 (optional)  : birdview (top-down) and freelook (3D) windows

The visualization world frame is anchored at the drone launch position (same as
FAST-LIO camera_init / PX4 ENU origin). Gazebo truth odom is shifted by the
spawn offset so /gt_path aligns with the FAST-LIO cloud and SUPER cloud in RViz.
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
    """Locate the workspace containing src/navigation/config."""
    override = os.environ.get("YUNGU_SIM_CONFIG")
    if override:
        simulation_config = Path(override).resolve()
        if simulation_config.is_file():
            return simulation_config.parents[3]
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "navigation" / "config" / "visualization.yaml").is_file():
            return parent
    return Path(__file__).resolve().parents[4]


def generate_launch_description():
    project_root = _find_workspace_root()
    config_path = str(project_root / 'src' / 'navigation' / 'config' / 'visualization.yaml')

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

    # Birdview overlay (src/navigation/config/visualization.yaml; disabled by default).
    bv_enabled = str(_cfg(config_path, 'birdview.enabled', True)).lower() == 'true'
    bv_image = _cfg(config_path, 'birdview.image', 'resources/yungu_birdview.png')
    if not os.path.isabs(bv_image):
        cand = os.path.join(project_root, bv_image)
        bv_image = cand if os.path.isfile(cand) else bv_image

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
            package='visualization', executable='birdview_publisher.py',
            name='birdview_publisher', output='screen',
            condition=IfCondition(str(bv_enabled).lower()),
            parameters=[{
                'image_path': bv_image,
                'topic': _cfg(config_path, 'birdview.topic', '/birdview_cloud'),
                'frame_id': _cfg(config_path, 'birdview.frame_id', world_frame),
                'extent_x': float(_cfg(config_path, 'birdview.extent_x', 500.0)),
                'extent_y': float(_cfg(config_path, 'birdview.extent_y', 300.0)),
                'z': float(_cfg(config_path, 'birdview.z', 0.0)),
                'offset_x': float(_cfg(config_path, 'birdview.offset_x', 0.0)),
                'offset_y': float(_cfg(config_path, 'birdview.offset_y', 0.0)),
                'yaw': float(_cfg(config_path, 'birdview.yaw', 0.0)),
                'max_points': int(_cfg(config_path, 'birdview.max_points', 1500000)),
                'republish_period': float(_cfg(config_path, 'birdview.republish_period', 10.0)),
            }],
        ),
        Node(
            package='visualization', executable='visual_tf', name='visual_tf',
            output='screen',
            parameters=[{
                'odom_topic': _cfg(config_path, 'visual_tf.odom_topic', '/gz/odom_super'),
                'world_frame': world_frame,
                'camera_init_frame': _cfg(config_path, 'frames.camera_init', 'camera_init'),
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
            package='visualization', executable='fastlio_visual.py',
            name='fastlio_visual', output='screen',
            parameters=[{
                'cloud_in_topic': _cfg(config_path, 'fastlio_visual.cloud_in_topic',
                                       '/cloud_registered'),
                'cloud_out_topic': _cfg(config_path, 'fastlio_visual.cloud_out_topic',
                                        '/fastlio_cloud'),
                'odom_in_topic': _cfg(config_path, 'fastlio_visual.odom_in_topic',
                                      '/Odometry'),
                'odom_out_topic': _cfg(config_path, 'fastlio_visual.odom_out_topic',
                                       '/fastlio_odom'),
                'world_frame': world_frame,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('visualization'), 'rviz',
                _cfg(config_path, 'rviz.birdview_config', 'birdview.rviz')])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_freelook', output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('visualization'), 'rviz',
                _cfg(config_path, 'rviz.freelook_config', 'freelook.rviz')])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
