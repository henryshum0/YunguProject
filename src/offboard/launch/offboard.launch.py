import importlib
import os
import re
import sys
import tempfile
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
            # The two side LiDARs are fused by lidar_merge into a base_link
            # cloud; super_bridge consumes that fused cloud now.
            default_cloud_in = f'/{model}/scan/points_fused'
        except Exception as exc:  # noqa: BLE001 - keep the launch working
            print(f'[offboard.launch] WARNING: could not read {sim_config_path}: '
                  f'{exc}; falling back to {default_cloud_in}', file=sys.stderr)

    # Project root (ancestor containing config/simulation.yaml).
    project_root = _find_project_root()

    # Offboard tuning params live in config/offboard.yaml (read via the shared
    # config/sim_config.py helper). ROS2 topic names are deliberately NOT here;
    # they stay as launch args (cmd_topic, odom_topic, cloud_*, *_topic, ...).
    offboard_cfg_path = None
    if project_root is not None:
        _of_cand = project_root / 'config' / 'offboard.yaml'
        if _of_cand.is_file():
            offboard_cfg_path = str(_of_cand)

    def _offboard_cfg(key, default):
        if sim_config is None or offboard_cfg_path is None:
            return default
        try:
            return sim_config.get_value(offboard_cfg_path,
                                        f'offboard.{key}', default)
        except Exception:  # noqa: BLE001 - keep the launch working
            return default

    # SUPER planner config: a file name (or absolute path) from config/offboard.yaml.
    # A bare name is resolved against config/super_planner/ first; otherwise it
    # stays bare and fsm_node resolves it against super_planner/config/.
    planner_cfg_name = str(_offboard_cfg('planner_config', 'gazebo-smooth.yaml'))
    default_planner_config = planner_cfg_name
    if project_root is not None:
        planner_cfg = project_root / 'config' / 'super_planner' / planner_cfg_name
        if planner_cfg.is_file():
            default_planner_config = str(planner_cfg)

    # Offboard state-machine tuning defaults (config/offboard.yaml).
    cfg_update_rate = str(_offboard_cfg('update_rate', 100.0))
    cfg_planner_cmd_hz = str(_offboard_cfg('planner_cmd_hz', 10.0))
    cfg_planner_enter_delay = str(_offboard_cfg('planner_enter_delay', 0.5))
    cfg_planner_exit_delay = str(_offboard_cfg('planner_exit_delay', 1.0))
    cfg_arm_wait = str(_offboard_cfg('arm_wait', 2.0))
    cfg_default_height = str(_offboard_cfg('default_height', 5.0))
    cfg_landing_vel = str(_offboard_cfg('landing_vel', 0.5))
    cfg_landing_z = str(_offboard_cfg('landing_z', 0.15))
    cfg_waypoint_reached_dist = str(_offboard_cfg('waypoint_reached_dist', 0.5))
    cfg_waypoint_hold_time = str(_offboard_cfg('waypoint_hold_time', 2.0))

    # Planner-config overrides applied by the launch (SUPER reads these from
    # its YAML file, not ROS params). A temporary copy of the planner config is
    # generated and fsm_node is pointed at it, only when something changes.
    fsm_config_path = default_planner_config

    # Master visualization switch (config/offboard.yaml): controls RViz windows,
    # the birdview overlay, and SUPER's marker publishing.
    visualization = _offboard_cfg('visualization', True)
    if isinstance(visualization, str):
        visualization = visualization.strip().lower() in ('1', 'true', 'yes')
    cfg_visualization = 'true' if visualization else 'false'

    # Goal height -> fsm.click_height.
    goal_height = _offboard_cfg('goal_height', None)
    if goal_height is not None:
        try:
            goal_height = float(goal_height)
        except (TypeError, ValueError):
            goal_height = None

    if (goal_height is not None or not visualization) and os.path.isfile(default_planner_config):
        try:
            with open(default_planner_config, encoding='utf-8') as f:
                text = f.read()
            new_text = text
            if goal_height is not None:
                m = re.search(r'click_height\s*:\s*([^\s#]+)', new_text)
                cur = float(m.group(1)) if m else None
                if cur is None or abs(cur - goal_height) > 1e-9:
                    new_text, _ = re.subn(
                        r'(click_height\s*:\s*)[^\s#]+',
                        r'\g<1>' + repr(goal_height), new_text, count=1)
            if not visualization:
                new_text, _ = re.subn(
                    r'(visualization_en\s*:\s*)[^\s#]+',
                    r'\g<1>false', new_text, count=1)
            if new_text != text:
                out = os.path.join(
                    tempfile.gettempdir(),
                    f'yungu_planner_{os.path.basename(default_planner_config)}')
                with open(out, 'w', encoding='utf-8') as f:
                    f.write(new_text)
                fsm_config_path = out
                print(f'[offboard.launch] planner config overrides -> {out}',
                      file=sys.stderr)
        except Exception as exc:  # noqa: BLE001 - keep the launch working
            print(f'[offboard.launch] WARNING: could not apply planner config '
                  f'overrides ({exc}); using original planner config',
                  file=sys.stderr)

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

    default_rviz_config = 'birdview.rviz'

    return LaunchDescription([
        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument('update_rate', default_value=cfg_update_rate,
                              description='State machine update rate [Hz]'),
        DeclareLaunchArgument('planner_cmd_hz', default_value=cfg_planner_cmd_hz,
                              description='Cmd rate threshold for planner hand-over [Hz]'),
        DeclareLaunchArgument('planner_enter_delay', default_value=cfg_planner_enter_delay,
                              description='Delay before entering planner mode [s]'),
        DeclareLaunchArgument('planner_exit_delay', default_value=cfg_planner_exit_delay,
                              description='Delay before exiting planner mode [s]'),
        DeclareLaunchArgument('arm_wait', default_value=cfg_arm_wait,
                              description='Wait after arming before OFFBOARD [s]'),
        DeclareLaunchArgument('default_height', default_value=cfg_default_height,
                              description='NED hover height after OFFBOARD (negative = up) [m]'),
        DeclareLaunchArgument('takeoff_duration', default_value='10.0',
                              description='Duration of the smooth takeoff trajectory [s]'),
        DeclareLaunchArgument('landing_vel', default_value=cfg_landing_vel,
                              description='Landing descent velocity [m/s]'),
        DeclareLaunchArgument('landing_z', default_value=cfg_landing_z,
                              description='Final landing height [m]'),
        DeclareLaunchArgument('cmd_topic', default_value='/planning/pos_cmd'),
        DeclareLaunchArgument('cloud_in_topic', default_value=default_cloud_in,
                              description='Lidar cloud for super_bridge (base_link '
                                          'frame, both side LiDARs fused); defaults '
                                          'to /<model>/scan/points_fused'),
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
                              description='Topic the offboard node uses to hand the current '
                                          'waypoint/goal to SUPER (reserved)'),
        DeclareLaunchArgument('waypoint_topic', default_value='/waypoint_pose',
                              description='Waypoint ingestion topic: publish waypoints here '
                                          '(e.g. re-target the RViz 2D Goal Pose tool)'),
        DeclareLaunchArgument('waypoint_buffer_topic', default_value='/waypoint_buffer',
                              description='Topic where the goal marker node forwards buffered '
                                          'waypoints for the offboard node'),
        DeclareLaunchArgument('waypoint_marker_topic', default_value='/waypoint_markers',
                              description='Topic where the offboard node publishes the waypoint '
                                          'buffer state as a MarkerArray (regularly)'),
        DeclareLaunchArgument('waypoint_reached_dist', default_value=cfg_waypoint_reached_dist,
                              description='Horizontal distance [m] to consider a waypoint reached'),
        DeclareLaunchArgument('waypoint_hold_time', default_value=cfg_waypoint_hold_time,
                              description='Hold time [s] between reaching a waypoint and starting '
                                          'the next one'),
        DeclareLaunchArgument('rviz', default_value=cfg_visualization,
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
        DeclareLaunchArgument('planner_config', default_value=fsm_config_path,
                              description='SUPER planner config (absolute path under '
                                          'config/super_planner/, or a file name in '
                                          'super_planner/config/)'),
        DeclareLaunchArgument('birdview', default_value=cfg_visualization,
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
                'takeoff_duration': LaunchConfiguration('takeoff_duration'),
                'landing_vel': LaunchConfiguration('landing_vel'),
                'landing_z': LaunchConfiguration('landing_z'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'local_pos_topic': LaunchConfiguration('local_pos_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'waypoint_buffer_topic': LaunchConfiguration('waypoint_buffer_topic'),
                'waypoint_reached_dist': LaunchConfiguration('waypoint_reached_dist'),
                'waypoint_hold_time': LaunchConfiguration('waypoint_hold_time'),
                'waypoint_marker_topic': LaunchConfiguration('waypoint_marker_topic'),
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
        # Goal marker node (waypoint ingestion -> waypoint buffer)
        #   - subscribes /waypoint_pose (user waypoints, e.g. RViz 2D Goal Pose
        #     re-targeted to this topic)
        #   - forwards the waypoints to /waypoint_buffer for the offboard node
        #     to buffer and fly (offboard visualizes the buffer on
        #     /waypoint_markers)
        # ------------------------------------------------------------------
        Node(
            package='offboard',
            executable='goal_marker_node',
            name='goal_marker_node',
            output='screen',
            parameters=[{
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'waypoint_buffer_topic': LaunchConfiguration('waypoint_buffer_topic'),
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
