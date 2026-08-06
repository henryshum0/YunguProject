#!/usr/bin/env python3
"""
fastlio_offboard.launch.py — Integrated launch for FAST-LIO + PX4 offboard.

Nodes launched:
  1. fastlio_mapping        — FAST-LIO (LiDAR-inertial odometry)
  2. fastlio_px4_bridge     — ENU→NED converter + PX4 EKF2 visual odometry
  3. offboard_node          — PX4 offboard state machine (auto takeoff/land)
  4. fsm_node               — SUPER planner (trajectory planning)

Usage:
  # From the workspace root:
  ros2 launch "$(pwd)/temp/fastlio_offboard.launch.py"

  # With custom configs:
  ros2 launch "$(pwd)/temp/fastlio_offboard.launch.py" \
      fastlio_config:=mid360.yaml \
      planner_config:=fastlio_planner.yaml

  # Without RViz:
  ros2 launch "$(pwd)/temp/fastlio_offboard.launch.py" rviz:=false

Prerequisites:
  - PX4 SITL running (start_sim.sh) or real PX4 on the flight controller
  - MicroXRCEAgent running (for real PX4) or started by start_sim.sh
  - LiDAR driver publishing /livox/lidar + /livox/imu (for real hardware)
  - Workspace sourced: source install/setup.bash
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


# -- Resolve temp/ directory (project root / temp) --------------------------
_TEMP_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_TEMP_DIR)


def _resolve_temp_script(name: str) -> str:
    """Absolute path to a script in temp/."""
    return os.path.join(_TEMP_DIR, name)


def generate_launch_description():
    # -- FAST-LIO package paths --------------------------------------------
    fastlio_share = get_package_share_directory("fast_lio")
    fastlio_default_config = os.path.join(fastlio_share, "config")

    # -- Offboard package paths ---------------------------------------------
    offboard_share = get_package_share_directory("offboard")
    offboard_default_rviz = os.path.join(offboard_share, "rviz", "x500.rviz")

    # -- SUPER planner config path ------------------------------------------
    super_planner_share = get_package_share_directory("super_planner")

    # ======================================================================
    #  Launch arguments
    # ======================================================================
    args = [
        # --- FAST-LIO -----------------------------------------------------
        DeclareLaunchArgument(
            "fastlio_config_path",
            default_value=fastlio_default_config,
            description="Directory containing FAST-LIO YAML configs",
        ),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value="mid360.yaml",
            description="FAST-LIO config file name (in fastlio_config_path/)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock",
        ),
        # --- Planner -------------------------------------------------------
        DeclareLaunchArgument(
            "planner_config",
            default_value=_resolve_temp_script("fastlio_planner.yaml"),
            description="Path to SUPER planner YAML config",
        ),
        # --- Bridge --------------------------------------------------------
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/Odometry",
            description="FAST-LIO odometry topic to subscribe",
        ),
        DeclareLaunchArgument(
            "ev_topic",
            default_value="/fmu/in/vehicle_visual_odometry",
            description="PX4 external vision odometry topic to publish",
        ),
        DeclareLaunchArgument(
            "republish_odom_topic",
            default_value="",  # disabled by default
            description="If set, also republish nav_msgs/Odometry on this topic",
        ),
        # --- Offboard ------------------------------------------------------
        DeclareLaunchArgument("update_rate", default_value="50.0"),
        DeclareLaunchArgument("planner_cmd_hz", default_value="10.0"),
        DeclareLaunchArgument("planner_enter_delay", default_value="0.5"),
        DeclareLaunchArgument("planner_exit_delay", default_value="1.0"),
        DeclareLaunchArgument("arm_wait", default_value="2.0"),
        DeclareLaunchArgument("offboard_wait", default_value="3.0"),
        DeclareLaunchArgument("takeoff_height", default_value="1.5"),
        DeclareLaunchArgument("takeoff_vel", default_value="1.0"),
        DeclareLaunchArgument("landing_vel", default_value="0.5"),
        DeclareLaunchArgument("landing_z", default_value="0.15"),
        DeclareLaunchArgument(
            "local_pos_topic",
            default_value="/fmu/out/vehicle_local_position_v1",
        ),
        # --- RViz ----------------------------------------------------------
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=offboard_default_rviz),
    ]

    nodes = []

    # ======================================================================
    #  1. FAST-LIO mapping
    # ======================================================================
    nodes.append(
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    LaunchConfiguration("fastlio_config_path"),
                    LaunchConfiguration("fastlio_config"),
                ]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        )
    )

    # ======================================================================
    #  2. FAST-LIO → PX4 bridge  (delayed start so FAST-LIO initializes)
    # ======================================================================
    bridge_node = Node(
        package="",  # standalone script — not in a ROS package
        executable=sys.executable,  # python3
        arguments=[_resolve_temp_script("fastlio_px4_bridge.py")],
        name="fastlio_px4_bridge",
        output="screen",
        parameters=[{
            "odom_topic": LaunchConfiguration("odom_topic"),
            "ev_topic": LaunchConfiguration("ev_topic"),
            "republish_odom_topic": LaunchConfiguration("republish_odom_topic"),
        }],
    )
    # Give FAST-LIO 3 seconds to initialize before the bridge starts.
    nodes.append(TimerAction(period=3.0, actions=[bridge_node]))

    # ======================================================================
    #  3. Offboard state machine
    # ======================================================================
    nodes.append(
        Node(
            package="offboard",
            executable="offboard_node",
            name="offboard",
            output="screen",
            parameters=[{
                "update_rate": LaunchConfiguration("update_rate"),
                "planner_cmd_hz": LaunchConfiguration("planner_cmd_hz"),
                "planner_enter_delay": LaunchConfiguration("planner_enter_delay"),
                "planner_exit_delay": LaunchConfiguration("planner_exit_delay"),
                "arm_wait": LaunchConfiguration("arm_wait"),
                "offboard_wait": LaunchConfiguration("offboard_wait"),
                "takeoff_height": LaunchConfiguration("takeoff_height"),
                "takeoff_vel": LaunchConfiguration("takeoff_vel"),
                "landing_vel": LaunchConfiguration("landing_vel"),
                "landing_z": LaunchConfiguration("landing_z"),
                "cmd_topic": "/planning/pos_cmd",
                "local_pos_topic": LaunchConfiguration("local_pos_topic"),
                "goal_topic": "/goal_pose",
                "goal_marker_topic": "/goal_marker",
            }],
        )
    )

    # ======================================================================
    #  4. SUPER planner (fsm_node)
    # ======================================================================
    nodes.append(
        Node(
            package="super_planner",
            executable="fsm_node",
            name="fsm_node",
            output="screen",
            parameters=[{
                "config_name": LaunchConfiguration("planner_config"),
            }],
        )
    )

    # ======================================================================
    #  5. RViz (optional)
    # ======================================================================
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return LaunchDescription(args + nodes)
