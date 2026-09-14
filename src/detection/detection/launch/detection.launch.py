"""detection.launch — bring up the simulated detection loop.

Launches (settings read from src/detection/config/):
  - spawn_targets     : one-shot, puts the ground-truth targets into the running
                        Gazebo world (skip with spawn:=false)
  - mock_detector     : simulation-only detector, publishes Detection2DArray
  - target_localizer  : back-projects those detections onto the ground plane
  - detection_overlay : debug view, the camera stream with boxes drawn on it

Run it after the simulation and the sensor bridge are up. On the real vehicle
only the mock_detector node is replaced; target_localizer and the configuration
stay as they are, so start it with mock:=false.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_MESH = 'VisionFlow-PX4/Tools/simulation/gz/worlds/yungu_collider.stl'


def _find_workspace_root() -> Path:
    """Locate the workspace that holds the detection configuration."""
    override = os.environ.get('YUNGU_DETECTION_CONFIG')
    if override:
        candidate = Path(override).resolve()
        if candidate.is_file():
            return candidate.parents[2]
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'detection' / 'config' / 'detection.yaml').is_file():
            return parent
    return Path.cwd()


def generate_launch_description():
    share = Path(get_package_share_directory('detection'))
    config_dir = share / 'config'
    models_dir = share / 'models'
    workspace_root = _find_workspace_root()
    # The collision mesh lives in the PX4 submodule, outside any install prefix,
    # so it is resolved here and passed as an absolute path.
    collision_mesh = workspace_root / COLLISION_MESH

    detection_config = LaunchConfiguration('detection_config')
    targets_config = LaunchConfiguration('targets_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'detection_config', default_value=str(config_dir / 'detection.yaml'),
            description='Shared detection contract: frames, camera model, topics, localizer.'),
        DeclareLaunchArgument(
            'mock_config', default_value=str(config_dir / 'mock_detector.yaml'),
            description='Simulation-only mock detector settings and error model.'),
        DeclareLaunchArgument(
            'targets_config', default_value=str(config_dir / 'targets.yaml'),
            description='Ground-truth targets shared by the spawner and the mock detector.'),
        DeclareLaunchArgument(
            'collision_mesh', default_value=str(collision_mesh),
            description='World collision mesh used for line-of-sight checks.'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='Gazebo world to spawn targets into (default: simulation.yaml).'),
        DeclareLaunchArgument(
            'spawn', default_value='true',
            description='Spawn the ground-truth targets into the running Gazebo world.'),
        DeclareLaunchArgument(
            'spawn_wait', default_value='60.0',
            description=('Seconds to wait for the Gazebo world before spawning, so this can be '
                         'launched together with the simulator rather than after it.')),
        DeclareLaunchArgument(
            'mock', default_value='true',
            description='Start the mock detector. Set false when a real detector publishes.'),
        DeclareLaunchArgument(
            'localizer', default_value='true',
            description='Start the detection-to-world-position node.'),
        DeclareLaunchArgument(
            'overlay', default_value='true',
            description='Start the debug overlay that draws boxes on the camera stream.'),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('spawn')),
            cmd=[
                str(Path(get_package_prefix('detection')) / 'lib' / 'detection' / 'spawn_targets'),
                '--detection-config', detection_config,
                '--targets-config', targets_config,
                '--models-dir', str(models_dir),
                '--world', LaunchConfiguration('world'),
                '--wait-sec', LaunchConfiguration('spawn_wait'),
            ],
            output='screen',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('mock')),
            package='detection', executable='mock_detector_node', name='mock_detector',
            output='screen',
            parameters=[{
                'detection_config': detection_config,
                'mock_config': LaunchConfiguration('mock_config'),
                'targets_config': targets_config,
                'collision_mesh': LaunchConfiguration('collision_mesh'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('localizer')),
            package='detection', executable='target_localizer_node', name='target_localizer',
            output='screen',
            parameters=[{
                'detection_config': detection_config,
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('overlay')),
            package='detection', executable='detection_overlay_node', name='detection_overlay',
            output='screen',
            parameters=[{
                'detection_config': detection_config,
            }],
        ),
    ])
