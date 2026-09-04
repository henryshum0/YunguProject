"""offboard.launch — bring up the perception + planning + offboard layer.

This launch is intentionally thin: it reads all tuning parameters from
src/navigation/config/offboard/offboard_fsm.yaml and all inter-module topics
from src/navigation/config/offboard/topics.yaml, then configures and starts
the nodes:

  - optional fastlio_mapping + fastlio_handler (FAST-LIO + PX4 visual-odometry bridge)
  - offboard_node (state machine)
  - super_bridge (PX4 odom + fused cloud -> SUPER world cloud/odom)
  - goal_marker_node (waypoint ingestion + marking)
  - fsm_node (super_planner)

The Gazebo sensor bridge (gz_sensor_interface) and the visualization layer
(visualization) are launched separately.
"""
import os
import re
import sys
import tempfile
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _find_navigation_config_dir():
    """Locate the workspace navigation configuration directory."""
    env = os.environ.get('YUNGU_SIM_CONFIG')
    if env:
        p = Path(env)
        if p.is_file():
            return p.parent
    for parent in Path(__file__).resolve().parents:
        config_dir = parent / 'src' / 'navigation' / 'config'
        if (config_dir / 'simulation.yaml').is_file():
            return config_dir
    return None


def _load_yaml(path):
    """Load a YAML file into a dict; return {} on any error."""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:  # noqa: BLE001 - keep the launch working
        print(f'[offboard.launch] WARNING: could not read {path}: {exc}',
              file=sys.stderr)
        return {}


def _dget(d, key, default=None):
    """Dot-path lookup in a nested dict (e.g. 'offboard_fsm.update_rate')."""
    node = d
    for part in key.split('.'):
        if not isinstance(node, dict):
            return default
        node = node.get(part)
    return default if node is None else node


def generate_launch_description():
    navigation_config_dir = _find_navigation_config_dir()
    cfg_dir = navigation_config_dir / 'offboard' if navigation_config_dir else None

    # ---- Load all config once -------------------------------------------------
    fsm = _load_yaml(cfg_dir / 'offboard_fsm.yaml').get('offboard_fsm', {}) if cfg_dir else {}
    topics = _load_yaml(cfg_dir / 'topics.yaml') if cfg_dir else {}
    sim = _load_yaml(navigation_config_dir / 'simulation.yaml') if navigation_config_dir else {}
    model = sim.get('model', 'swan_gamma_v2')

    def cfg(key, default=None):
        return _dget(fsm, key, default)

    def topic(key, default=None):
        return _dget(topics, key, default)

    # ---- Derived paths --------------------------------------------------------
    # FAST-LIO params: bare name resolves against src/navigation/config/offboard/.
    fastlio_cfg_name = str(cfg('fastlio_config', 'fastlio_swan_gamma_effect.yaml'))
    default_fastlio_config = fastlio_cfg_name
    if navigation_config_dir is not None:
        cand = cfg_dir / fastlio_cfg_name
        if cand.is_file():
            default_fastlio_config = str(cand)

    # SUPER planner config: bare name resolves against
    # src/navigation/config/offboard/super_planner/.
    planner_cfg_name = str(cfg('planner_config', 'gazebo-smooth.yaml'))
    default_planner_config = planner_cfg_name
    if navigation_config_dir is not None:
        cand = cfg_dir / 'super_planner' / planner_cfg_name
        if cand.is_file():
            default_planner_config = str(cand)

    # Fused cloud read by super_bridge (gz_sensor_interface always fuses).
    default_cloud_in = str(cfg('cloud_in_topic', f'/{model}/scan/points_fused'))

    # ---- Planner-config overrides ----------------------------------------------
    # goal_height is applied by goal_marker_node (it stamps the RViz 2D goal's z),
    # so it no longer overrides the SUPER planner's click_height here.
    #
    # SUPER's inter-module topics are sourced from
    # src/navigation/config/offboard/topics.yaml and
    # injected into the planner config so SUPER reads its topics centrally:
    #   fsm.click_goal_topic        <- super.in.goal_pose
    #   fsm.cmd_topic               <- super.out.pos_cmd
    #   rog_map.ros_callback.cloud_topic <- super.in.cloud
    #   rog_map.ros_callback.odom_topic  <- super_bridge.out_odom
    visualization = cfg('visualization', True)
    if isinstance(visualization, str):
        visualization = visualization.strip().lower() in ('1', 'true', 'yes')

    super_topics = {
        r'click_goal_topic\s*:': topic('super.in.goal_pose', '/goal_pose'),
        r'cmd_topic\s*:': topic('super.out.pos_cmd', '/planning/pos_cmd'),
        r'cloud_topic\s*:': topic('super.in.cloud', '/cloud_registered_px4'),
        r'odom_topic\s*:': topic('super.in.odom', '/lidar_slam/odom'),
    }

    fsm_config_path = default_planner_config
    if os.path.isfile(default_planner_config):
        try:
            with open(default_planner_config, encoding='utf-8') as f:
                text = f.read()
            new_text = text
            for pattern, val in super_topics.items():
                # Replace the value after "<key>:" (keep the key, overwrite the value).
                new_text, _ = re.subn(
                    r'(' + pattern + r')\s*[^\s#]+',
                    r'\g<1> ' + repr(val), new_text, count=1)
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
                print(f'[offboard_fsm.launch] planner config overrides -> {out}',
                      file=sys.stderr)
        except Exception as exc:  # noqa: BLE001 - keep the launch working
            print(f'[offboard_fsm.launch] WARNING: could not apply planner config '
                  f'overrides ({exc}); using original', file=sys.stderr)

    use_sim_time = 'true' if cfg('use_sim_time', True) else 'false'

    # ---- Launch description ----------------------------------------------------
    # Tuning parameters are read straight from
    # src/navigation/config/offboard/offboard_fsm.yaml
    # as native YAML types (float/int) so the nodes receive correctly-typed values.
    # Only the inter-module topics are exposed as launch args (overridable).
    num_args = {
        'update_rate': float(cfg('update_rate', 100.0)),
        'planner_cmd_hz': float(cfg('planner_cmd_hz', 10.0)),
        'planner_enter_delay': float(cfg('planner_enter_delay', 0.5)),
        'planner_exit_delay': float(cfg('planner_exit_delay', 1.0)),
        'arm_wait': float(cfg('arm_wait', 2.0)),
        'default_height': float(cfg('default_height', 5.0)),
        'landing_vel': float(cfg('landing_vel', 0.5)),
        'takeoff_vel': float(cfg('takeoff_vel', 0.5)),
        'landing_z': float(cfg('landing_z', 0.15)),
        'waypoint_reached_dist': float(cfg('waypoint_reached_dist', 0.5)),
        'waypoint_hold_time': float(cfg('waypoint_hold_time', 2.0)),
        'arm_retry_delay': float(cfg('arm_retry_delay', 5.0)),
        'arm_retry_max': int(cfg('arm_retry_max', 3)),
        'planner_reset_delay': float(cfg('planner_reset_delay', 5.0)),
        'yaw_align_thresh': float(cfg('yaw_align_thresh', 0.35)),
        'goal_height': float(cfg('goal_height', 5.0)),
    }
    # SUPER's world cloud + odom (/gz/point_cloud_super, /lidar_slam/odom) come
    # from gz_sensor_interface/super_lidar; only the offboard_fsm I/O topics below
    # need to be launch args here.
    topic_args = {
        'cmd_topic': str(topic('super.out.pos_cmd', '/planning/pos_cmd')),
        'goal_topic': str(topic('super.in.goal_pose', '/goal_pose')),
        'waypoint_queue_service': str(topic(
            'offboard_fsm.services.queue_waypoints', '/waypoint_buffer')),
        'clear_waypoints_service': str(topic(
            'offboard_fsm.services.clear_waypoints', '/waypoint_buffer/clear')),
        'cloud_in_topic': default_cloud_in,
    }

    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=val)
            for name, val in topic_args.items()
        ],
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument(
            'use_fastlio', default_value='true',
            description='Launch FAST-LIO and its PX4 visual-odometry bridge.'),
        DeclareLaunchArgument('fastlio_config', default_value=default_fastlio_config),
        DeclareLaunchArgument('planner_config', default_value=fsm_config_path),

        # Optional FAST-LIO layer; consumes fused cloud + IMU from
        # gz_sensor_interface, launched separately.
        # FAST-LIO reads its input topics (fused cloud + IMU) from topics.yaml;
        # these override the defaults inside fastlio_swan_gamma_effect.yaml.
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
             output='screen',
             condition=IfCondition(LaunchConfiguration('use_fastlio')),
             parameters=[
                 LaunchConfiguration('fastlio_config'),
                 {
                     # FAST-LIO reads these under the `common` namespace (it does
                     # not declare/use a top-level use_sim_time, so leave that out).
                     'common.lid_topic': topic('fastlio.in.cloud',
                                               '/swan_gamma_v2/scan/points_fused'),
                     'common.imu_topic': topic('fastlio.in.imu', '/livox/imu'),
                 },
             ]),

        # FAST-LIO -> PX4 visual odometry bridge (C++ replacement for the old
        # scripts/fastlio_px4_bridge.py).
        Node(package='offboard_fsm', executable='fastlio_handler',
             name='fastlio_handler', output='screen',
             condition=IfCondition(LaunchConfiguration('use_fastlio')),
             parameters=[{
                 'odom_topic': topic('fastlio.out.odometry', '/Odometry'),
                 'ev_topic': topic('fastlio.out.vehicle_visual_odometry',
                                   '/fmu/in/vehicle_visual_odometry'),
             }]),

        # PX4 offboard state machine.
        Node(
            package='offboard_fsm', executable='offboard_node', name='offboard',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                **num_args,
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'cloud_in_topic': LaunchConfiguration('cloud_in_topic'),
                'local_pos_topic': topic('offboard_fsm.in.local_pos',
                                          '/fmu/out/vehicle_local_position_v1'),
                'status_topic': topic('offboard_fsm.in.vehicle_status',
                                      '/fmu/out/vehicle_status_v4'),
                'land_detected_topic': topic('offboard_fsm.in.vehicle_land_detected',
                                             '/fmu/out/vehicle_land_detected'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'waypoint_queue_service': LaunchConfiguration('waypoint_queue_service'),
                'clear_waypoints_service': LaunchConfiguration('clear_waypoints_service'),
                'planner_state_topic': topic('super.out.planner_state', 'fsm/planner_state'),
                'goal_status_topic': topic('super.out.goal_status', 'fsm/goal_status'),
                'lio_state_topic': topic('fastlio.out.lio_state', 'fastlio/lio_state'),
                'planner_reset_service': topic('offboard_fsm.out.planner_reset',
                                               '/fsm_node/reset'),
                'takeoff_cmd_topic': topic('offboard_fsm.in.takeoff_cmd', '/takeoff_cmd'),
                'land_cmd_topic': topic('offboard_fsm.in.land_cmd', '/land_cmd'),
            }],
        ),

        # SUPER's world cloud + odom are produced by gz_sensor_interface/super_lidar
        # (/gz/point_cloud_super + /lidar_slam/odom); offboard no longer runs a
        # super_bridge.

        # Goal marker node (waypoint ingestion + marking -> waypoint queue service).
        # Stamps the RViz 2D goal's height with goal_height before forwarding.
        Node(
            package='offboard_fsm', executable='goal_marker_node', name='goal_marker_node',
            output='screen',
            parameters=[{
                'waypoint_topic': topic('offboard_fsm.in.waypoint_pose', '/waypoint_pose'),
                'waypoint_queue_service': LaunchConfiguration('waypoint_queue_service'),
                'waypoint_marker_topic': topic('offboard_fsm.out.waypoint_markers',
                                               '/waypoint_markers'),
                'waypoint_marker_rate': 10.0,
                'goal_height': num_args['goal_height'],
            }],
        ),

        # SUPER planner (fsm_node); config from <planner_config>.
        Node(
            package='super_planner', executable='fsm_node', name='fsm_node',
            output='screen',
            parameters=[{
                'config_name': LaunchConfiguration('planner_config'),
            }],
        ),
    ])
