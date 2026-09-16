"""Launch EGO-Planner and the PX4 adapter against the running Yungu simulation.

PX4/Gazebo, the GZ bridge, and ``gz_sensor_interface`` are intentionally not
started here.  They provide the configured world-frame cloud and odometry.
"""

from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mapping(value, name):
    if not isinstance(value, dict):
        raise RuntimeError(f"{name} must be a YAML mapping")
    return value


def _load(path: Path, name: str):
    if not path.is_file():
        raise RuntimeError(f"{name} configuration is missing: {path}")
    with path.open(encoding="utf-8") as stream:
        return _mapping(yaml.safe_load(stream) or {}, name)


def _nodes(context):
    config_dir = Path(LaunchConfiguration("navigation_config_dir").perform(context)).expanduser().resolve()
    topics = _load(config_dir / "topics.yaml", "topics")
    settings = _load(config_dir / "offboard_fsm.yaml", "offboard FSM")
    offboard_topics = _mapping(topics.get("offboard_fsm"), "offboard_fsm topics")
    ego_topics = _mapping(topics.get("ego_planner"), "ego_planner topics")
    offboard_in = _mapping(offboard_topics.get("in"), "offboard_fsm.in")
    offboard_out = _mapping(offboard_topics.get("out"), "offboard_fsm.out")
    offboard_services = _mapping(offboard_topics.get("services"), "offboard_fsm.services")
    offboard = _mapping(settings.get("offboard_fsm"), "offboard_fsm settings")
    ego = _mapping(settings.get("ego_planner"), "ego_planner settings")

    ego_parameters = {
        "fsm/flight_type": 1,
        "fsm/thresh_replan_time": 1.0,
        "fsm/thresh_no_replan_meter": 1.0,
        "fsm/planning_horizon": ego["planning_horizon_m"],
        "fsm/planning_horizen_time": 3.0,
        "fsm/emergency_time": 1.0,
        "fsm/realworld_experiment": False,
        "fsm/fail_safe": True,
        "grid_map/resolution": ego["grid_resolution_m"],
        "grid_map/map_size_x": ego["map_size_x_m"],
        "grid_map/map_size_y": ego["map_size_y_m"],
        "grid_map/map_size_z": ego["map_size_z_m"],
        "grid_map/local_update_range_x": ego["local_update_range_x_m"],
        "grid_map/local_update_range_y": ego["local_update_range_y_m"],
        "grid_map/local_update_range_z": ego["local_update_range_z_m"],
        "grid_map/obstacles_inflation": ego["obstacle_inflation_m"],
        "grid_map/local_map_margin": 2,
        "grid_map/ground_height": ego["ground_height_m"],
        "grid_map/use_depth_filter": False,
        "grid_map/virtual_ceil_height": ego["virtual_ceiling_height_m"],
        "grid_map/visualization_truncate_height": ego["virtual_ceiling_height_m"],
        "grid_map/pose_type": 2,
        "grid_map/frame_id": offboard["frame_id"],
        "manager/max_vel": ego["max_velocity_mps"],
        "manager/max_acc": ego["max_acceleration_mps2"],
        "manager/max_jerk": ego["max_jerk_mps3"],
        "manager/control_points_distance": 0.4,
        "manager/feasibility_tolerance": 0.05,
        "manager/planning_horizon": ego["planning_horizon_m"],
        "manager/use_distinctive_trajs": False,
        "manager/drone_id": 0,
        "optimization/lambda_smooth": 1.0,
        "optimization/lambda_collision": 0.5,
        "optimization/lambda_feasibility": 0.1,
        "optimization/lambda_fitness": 1.0,
        "optimization/dist0": 0.5,
        "optimization/swarm_clearance": 0.5,
        "optimization/max_vel": ego["max_velocity_mps"],
        "optimization/max_acc": ego["max_acceleration_mps2"],
        "prediction/obj_num": 0,
        "prediction/lambda": 1.0,
        "prediction/predict_rate": 1.0,
    }
    fsm_parameters = {
        **offboard,
        "local_position_topic": offboard_in["local_position"],
        "vehicle_status_topic": offboard_in["vehicle_status"],
        "land_detected_topic": offboard_in["land_detected"],
        "ego_command_topic": offboard_in["ego_position_command"],
        "ego_state_topic": offboard_in["ego_state"],
        "ego_goal_topic": offboard_out["ego_goal"],
        "queue_status_topic": offboard_out["waypoint_queue_status"],
        "offboard_mode_topic": offboard_out["offboard_control_mode"],
        "trajectory_setpoint_topic": offboard_out["trajectory_setpoint"],
        "vehicle_command_topic": offboard_out["vehicle_command"],
        "queue_service": offboard_services["queue_waypoints"],
        "clear_service": offboard_services["clear_waypoints"],
        "takeoff_service": offboard_services["takeoff"],
        "land_service": offboard_services["land"],
    }
    return [
        Node(
            package="ego_planner", executable="ego_planner_node", name="ego_planner",
            output="screen", parameters=[ego_parameters], remappings=[
                ("odom_world", ego_topics["odometry"]),
                ("grid_map/odom", ego_topics["odometry"]),
                ("grid_map/cloud", ego_topics["cloud"]),
                ("/move_base_simple/goal", ego_topics["goal"]),
                ("planning/bspline", ego_topics["bspline"]),
                ("state", ego_topics["state"]),
            ],
        ),
        Node(
            package="ego_planner", executable="traj_server", name="ego_traj_server",
            output="screen", parameters=[{"traj_server/time_forward": 1.0}], remappings=[
                ("planning/bspline", ego_topics["bspline"]),
                ("/position_cmd", ego_topics["position_command"]),
            ],
        ),
        Node(
            package="offboard_fsm", executable="offboard_node", name="offboard",
            output="screen", parameters=[fsm_parameters],
        ),
    ]


def generate_launch_description():
    workspace = Path(__file__).resolve().parents[2]
    return LaunchDescription([
        DeclareLaunchArgument(
            "navigation_config_dir",
            default_value=str(workspace / "src" / "navigation" / "config" / "offboard"),
            description="Directory containing topics.yaml and offboard_fsm.yaml",
        ),
        OpaqueFunction(function=_nodes),
    ])
