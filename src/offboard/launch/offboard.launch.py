import importlib
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _find_project_root():
    """Locate the Yungu project root.

    The root is the closest ancestor of this launch file that contains
    config/simulation.yaml (works with the repo's symlink-install layout).
    Priority: $YUNGU_SIM_CONFIG env var (points at config/simulation.yaml),
    then the ancestor walk.
    """
    env = os.environ.get('YUNGU_SIM_CONFIG')
    if env:
        p = Path(env)
        if p.is_file():
            return p.parent.parent  # .../config/simulation.yaml -> project root
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'config' / 'simulation.yaml').is_file():
            return parent
    return None


def _find_sim_config():
    """Locate config/simulation.yaml (the Yungu simulation config)."""
    root = _find_project_root()
    if root is not None:
        cand = root / 'config' / 'simulation.yaml'
        if cand.is_file():
            return cand
    return None


def generate_launch_description():
    # The raw gz lidar topic follows the configured model, e.g. model=swan_gamma_v2
    # -> /swan_gamma_v2/scan/points. Read config/simulation.yaml (via the shared
    # config/sim_config.py helper) so the launch automatically tracks the model
    # selected there. Override with cloud_in_topic:=... if needed.
    default_cloud_in = '/x500_lidar/scan/points'
    sim_config_path = _find_sim_config()
    sim_config = None
    if sim_config_path is not None:
        try:
            # Reuse config/sim_config.py (sits next to simulation.yaml).
            sys.path.insert(0, str(sim_config_path.parent))
            sim_config = importlib.import_module('sim_config')
            model = sim_config.get_value(sim_config_path, 'model', default='x500_lidar')
            default_cloud_in = f'/{model}/scan/points'
        except Exception as exc:  # noqa: BLE001 - keep the launch working
            print(f'[offboard.launch] WARNING: could not read {sim_config_path}: '
                  f'{exc}; falling back to {default_cloud_in}', file=sys.stderr)

    # SUPER planner config. It lives with the rest of the project config in
    # config/super_planner/ (gazebo-smooth.yaml); fsm_node accepts an absolute
    # path via config_name. Fall back to a bare file name resolved against
    # super_planner/config/ if the project-level file is missing.
    default_planner_config = 'gazebo-smooth.yaml'
    project_root = _find_project_root()
    if project_root is not None:
        planner_cfg = project_root / 'config' / 'super_planner' / 'gazebo-smooth.yaml'
        if planner_cfg.is_file():
            default_planner_config = str(planner_cfg)

    # Birdview aerial overlay (top-down map reference in RViz). The tuneable
    # transform parameters live in config/birdview.yaml (read via the shared
    # config/sim_config.py helper); the PNG is published as a flat colored
    # point cloud on the ground plane.
    birdview_cfg_path = None
    if project_root is not None:
        _bv_cand = project_root / 'config' / 'birdview.yaml'
        if _bv_cand.is_file():
            birdview_cfg_path = str(_bv_cand)

    def _birdview_cfg(key, default):
        if sim_config is None or birdview_cfg_path is None:
            return default
        try:
            return sim_config.get_value(birdview_cfg_path,
                                        f'birdview.{key}', default)
        except Exception:  # noqa: BLE001 - keep the launch working
            return default

    default_birdview_image = ''
    if project_root is not None:
        img = _birdview_cfg('image', 'resources/yungu_birdview.png')
        cand = project_root / img
        if not cand.is_file():
            cand = Path(img)
        if cand.is_file():
            default_birdview_image = str(cand)

    # RViz configs shipped with the offboard package.
    #   birdview.rviz (default): clean top-down planning view - aerial birdview
    #       overlay + occupied map + trajectory (no debug markers).
    #   freelook.rviz: full 3D debug view - corridors, trajectories, markers,
    #       free-rotate Orbit camera, no birdview overlay.
    #   x500.rviz: minimal sensor-only view (fallback).
    # rviz_config takes a bare file name resolved against the package rviz/
    # dir, e.g. rviz_config:=freelook.rviz.
    default_rviz_config = 'birdview.rviz'

    return LaunchDescription([
        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument('update_rate', default_value='100.0'),
        DeclareLaunchArgument('planner_cmd_hz', default_value='10.0',
                              description='Cmd rate threshold for planner hand-over [Hz]'),
        DeclareLaunchArgument('planner_enter_delay', default_value='0.5'),
        DeclareLaunchArgument('planner_exit_delay', default_value='1.0'),
        DeclareLaunchArgument('arm_wait', default_value='2.0'),
        DeclareLaunchArgument('default_height', default_value='5.0',
                              description='NED hover height after OFFBOARD (negative = up) [m]'),
        DeclareLaunchArgument('landing_vel', default_value='0.5'),
        DeclareLaunchArgument('landing_z', default_value='0.15'),
        DeclareLaunchArgument('cmd_topic', default_value='/planning/pos_cmd'),
        DeclareLaunchArgument('cloud_in_topic', default_value=default_cloud_in,
                              description='Raw gz lidar cloud (lidar_link frame); '
                                          'defaults to /<model>/scan/points from '
                                          'config/simulation.yaml'),
        DeclareLaunchArgument('odom_topic', default_value='/fmu/out/vehicle_odometry',
                              description='PX4 odometry topic (NED, converted to ENU by the bridge)'),
        DeclareLaunchArgument('cloud_registered_topic', default_value='/cloud_registered',
                              description='World-frame lidar cloud for SUPER'),
        DeclareLaunchArgument('lidar_slam_odom_topic', default_value='/lidar_slam/odom',
                              description='Odom topic republished for SUPER'),
        DeclareLaunchArgument('local_pos_topic',
                              default_value='/fmu/out/vehicle_local_position_v1',
                              description='PX4 local position topic (note the _v1 suffix)'),
        DeclareLaunchArgument('status_topic', default_value='/fmu/out/vehicle_status_v4',
                              description='PX4 vehicle status topic (note the _v4 suffix)'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='RViz 2D Goal Pose topic'),
        DeclareLaunchArgument('goal_marker_topic', default_value='/goal_marker',
                              description='Goal visualization marker topic'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch the RViz2 visualization windows'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Config for the planning window '
                                          '(birdview.rviz by default)'),
        DeclareLaunchArgument('rviz_freelook', default_value='true',
                              description='Launch the 3D debug (freelook) RViz '
                                          'window alongside the planning one'),
        DeclareLaunchArgument('rviz_freelook_config', default_value='freelook.rviz',
                              description='Config for the 3D debug window '
                                          '(freelook.rviz by default)'),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config,
                              description='SUPER planner config (absolute path under '
                                          'config/super_planner/, or a file name in '
                                          'super_planner/config/)'),
        DeclareLaunchArgument('birdview', default_value='true',
                              description='Publish the aerial birdview overlay (/birdview_cloud)'),
        DeclareLaunchArgument('birdview_image', default_value=default_birdview_image,
                              description='Birdview PNG path (default: resources/yungu_birdview.png)'),

        # ------------------------------------------------------------------
        # PX4 offboard state machine
        # ------------------------------------------------------------------
        Node(
            package='offboard',
            executable='offboard_node',
            name='offboard',
            output='screen',
            parameters=[{
                'update_rate': LaunchConfiguration('update_rate'),
                'planner_cmd_hz': LaunchConfiguration('planner_cmd_hz'),
                'planner_enter_delay': LaunchConfiguration('planner_enter_delay'),
                'planner_exit_delay': LaunchConfiguration('planner_exit_delay'),
                'arm_wait': LaunchConfiguration('arm_wait'),
                'default_height': LaunchConfiguration('default_height'),
                'landing_vel': LaunchConfiguration('landing_vel'),
                'landing_z': LaunchConfiguration('landing_z'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'local_pos_topic': LaunchConfiguration('local_pos_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
            }],
        ),

        # ------------------------------------------------------------------
        # Super bridge
        #   - subscribes to the raw gz lidar cloud (lidar_link frame) + PX4
        #     vehicle_odometry (NED)
        #   - converts the odom to ENU, transforms the points into the world
        #     frame and republishes as /cloud_registered (with /lidar_slam/odom)
        #     which is what SUPER's ROG-Map consumes (gazebo.yaml)
        # ------------------------------------------------------------------
        Node(
            package='offboard',
            executable='super_bridge',
            name='super_bridge',
            output='screen',
            parameters=[{
                'cloud_in_topic': LaunchConfiguration('cloud_in_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cloud_out_topic': LaunchConfiguration('cloud_registered_topic'),
                'odom_out_topic': LaunchConfiguration('lidar_slam_odom_topic'),
            }],
        ),

        # ------------------------------------------------------------------
        # Goal marker node (RViz 2D Goal Pose -> /goal_marker markers)
        # ------------------------------------------------------------------
        Node(
            package='offboard',
            executable='goal_marker_node',
            name='goal_marker_node',
            output='screen',
            parameters=[{
                'goal_topic': LaunchConfiguration('goal_topic'),
                'goal_marker_topic': LaunchConfiguration('goal_marker_topic'),
            }],
        ),

        # ------------------------------------------------------------------
        # SUPER planner (fsm_node)
        #   - loads config from <planner_config> (defaults to an absolute path
        #     under config/super_planner/; a bare file name resolves against
        #     super_planner/config/)
        #   - subscribes to /cloud_registered + /lidar_slam/odom (super_bridge)
        #   - publishes /planning/pos_cmd (consumed by the offboard node)
        #   - receives goals on /goal_pose (RViz "2D Goal Pose" tool)
        # ------------------------------------------------------------------
        Node(
            package='super_planner',
            executable='fsm_node',
            name='fsm_node',
            output='screen',
            parameters=[{
                'config_name': LaunchConfiguration('planner_config'),
            }],
        ),

        # ------------------------------------------------------------------
        # Birdview overlay (top-down map reference in RViz)
        #   - publishes resources/yungu_birdview.png as a flat colored point
        #     cloud on the ground plane (world frame) for waypoint planning
        # ------------------------------------------------------------------
        Node(
            package='offboard',
            executable='birdview_publisher.py',
            name='birdview_publisher',
            output='screen',
            parameters=[{
                'image_path': LaunchConfiguration('birdview_image'),
                'topic': _birdview_cfg('topic', '/birdview_cloud'),
                'frame_id': _birdview_cfg('frame_id', 'world'),
                # Coerce numerics: YAML ints (e.g. `z: -2`) would otherwise
                # type-clash with the node's float params and crash it.
                'extent_x': float(_birdview_cfg('extent_x', 500.0)),
                'extent_y': float(_birdview_cfg('extent_y', 300.0)),
                'z': float(_birdview_cfg('z', 0.0)),
                'offset_x': float(_birdview_cfg('offset_x', 0.0)),
                'offset_y': float(_birdview_cfg('offset_y', 0.0)),
                'yaw': float(_birdview_cfg('yaw', 0.0)),
                'max_points': int(_birdview_cfg('max_points', 3000000)),
                'republish_period': float(_birdview_cfg('republish_period', 10.0)),
            }],
            condition=IfCondition(LaunchConfiguration('birdview')),
        ),

        # ------------------------------------------------------------------
        # RViz visualization (two windows launched together)
        #   rviz2          : planning window (top-down birdview by default)
        #   rviz2_freelook : 3D debug window (free-rotate, no birdview)
        # ------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('offboard'), 'rviz',
                LaunchConfiguration('rviz_config')])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_freelook',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('offboard'), 'rviz',
                LaunchConfiguration('rviz_freelook_config')])],
            condition=IfCondition(PythonExpression([
                TextSubstitution(text="('"),
                LaunchConfiguration('rviz'),
                TextSubstitution(text="' == 'true') and ('"),
                LaunchConfiguration('rviz_freelook'),
                TextSubstitution(text="' == 'true')"),
            ])),
        ),
    ])
