"""Start the coverage planner together with the offboard FSM stack.

This is a workspace-level launch file and is intended to be run directly after
sourcing the YunguProject overlay. The included offboard launch continues to
load its controller, topic, and simulation settings from
``src/navigation/config/``. Coverage planner mission/map settings are supplied through
``config_file``.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the combined planner and offboard-FSM launch description."""
    coverage_share = Path(get_package_share_directory("coverage_planner"))
    offboard_share = Path(get_package_share_directory("offboard_fsm"))

    default_config_file = coverage_share / "config" / "yungu_planner.json"
    offboard_launch = offboard_share / "launch" / "offboard.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=str(default_config_file),
            description=(
                "Absolute path to a coverage_planner schema-1.2 JSON file. "
                "Its map_file is resolved relative to that JSON."
            ),
        ),
        DeclareLaunchArgument(
            "use_fastlio",
            default_value="true",
            description="Launch FAST-LIO and its PX4 visual-odometry bridge.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(offboard_launch)),
            launch_arguments={
                "use_fastlio": LaunchConfiguration("use_fastlio"),
            }.items(),
        ),
        Node(
            package="coverage_planner",
            executable="coverage_planner_node",
            name="coverage_planner",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
            }],
        ),
    ])
