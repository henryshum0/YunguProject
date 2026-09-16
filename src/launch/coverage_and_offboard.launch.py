"""Start coverage planning together with the EGO/PX4 navigation stack.

PX4/Gazebo, the GZ bridge, and ``gz_sensor_interface`` must already be
running. Coverage planning remains on-demand through its action interface.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the combined coverage planner and EGO/PX4 launch description."""
    coverage_share = Path(get_package_share_directory("coverage_planner"))
    default_config_file = coverage_share / "config" / "yungu_planner.json"
    workspace = Path(__file__).resolve().parents[2]
    navigation_launch = workspace / "src" / "launch" / "ego_single_drone.launch.py"
    default_navigation_config = workspace / "src" / "navigation" / "config" / "offboard"

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
            "navigation_config_dir",
            default_value=str(default_navigation_config),
            description="Directory containing EGO/PX4 topics.yaml and offboard_fsm.yaml.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation_launch)),
            launch_arguments={
                "navigation_config_dir": LaunchConfiguration("navigation_config_dir"),
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
