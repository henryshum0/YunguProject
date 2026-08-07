import importlib
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _find_sim_config():
    """Locate config/simulation.yaml (the Yungu simulation config).

    Priority: $YUNGU_SIM_CONFIG env var, then the closest ancestor of this
    launch file that contains config/simulation.yaml (works with the repo's
    symlink-install layout).
    """
    env = os.environ.get('YUNGU_SIM_CONFIG')
    if env:
        p = Path(env)
        if p.is_file():
            return p
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / 'config' / 'simulation.yaml'
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
    if sim_config_path is not None:
        try:
            # Reuse config/sim_config.py (sits next to simulation.yaml).
            sys.path.insert(0, str(sim_config_path.parent))
            _sim_cfg = importlib.import_module('sim_config')
            model = _sim_cfg.get_value(sim_config_path, 'model', default='x500_lidar')
            default_cloud_in = f'/{model}/scan/points'
        except Exception as exc:  # noqa: BLE001 - keep the launch working
            print(f'[offboard.launch] WARNING: could not read {sim_config_path}: '
                  f'{exc}; falling back to {default_cloud_in}', file=sys.stderr)

    # RViz config shipped with the offboard package.
    # Default to the full path/planning visualization (corridors, trajectories,
    # occupied map, TF, goal markers). Override with rviz_config:=x500.rviz
    # to fall back to the minimal sensor-only view.
    default_rviz_config = os.path.join(
        get_package_share_directory('offboard'), 'rviz', 'x500_lidar_paths.rviz')

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
                              description='Launch RViz2 visualization'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('planner_config', default_value='gazebo-smooth.yaml',
                              description='SUPER planner config (super_planner/config/)'),

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
        #   - loads config from super_planner/config/<planner_config>
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
        # RViz visualization (drone TF + lidar point cloud + 2D Goal Pose)
        # ------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
