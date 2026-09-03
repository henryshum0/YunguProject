"""Start the coverage planner and its default RViz visualization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_directory = Path(get_package_share_directory("coverage_planner"))
    default_config = share_directory / "config" / "example_planner.json"
    rviz_config = share_directory / "rviz" / "coverage_planner.rviz"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=str(default_config),
            description="Absolute path to the planner startup JSON.",
        ),
        DeclareLaunchArgument(
            "waypoints_topic",
            default_value="/coverage_planner/waypoints",
            description="Must match output_topics.waypoints in the startup JSON.",
        ),
        DeclareLaunchArgument(
            "markers_topic",
            default_value="/coverage_planner/markers",
            description="Must match output_topics.markers in the startup JSON.",
        ),
        Node(
            package="coverage_planner",
            executable="coverage_planner_node",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            remappings=[
                ("/coverage_planner/waypoints", LaunchConfiguration("waypoints_topic")),
                ("/coverage_planner/markers", LaunchConfiguration("markers_topic")),
            ],
            output="screen",
        ),
    ])
