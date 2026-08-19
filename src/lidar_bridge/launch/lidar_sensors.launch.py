"""lidar_sensors.launch.py — Launch the dual-side LiDAR time-fielding + fusion layer.

Replaces the manual `ros2 run` block that used to live in
utils/start_fastlio.sh (two add_time_field nodes + lidar_merge) with a single
launch file:

  ros2 launch lidar_bridge lidar_sensors.launch.py            # model=swan_gamma_v2
  ros2 launch lidar_bridge lidar_sensors.launch.py model:=x500_lidar

Data flow (mirrors the real hardware — one LiDAR per side, fused into one
base_link cloud for FAST-LIO; lidar_merge applies the mounting extrinsics):

  /<model>/scan_left/points  ─add_time_field→  /<model>/scan_left/points_timed  ─┐
                                                                                  ├─ lidar_merge → /<model>/scan/points_fused
  /<model>/scan_right/points ─add_time_field→  /<model>/scan_right/points_timed ─┘
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration("model", default="swan_gamma_v2")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="swan_gamma_v2",
            description="Simulation model name used to build the LiDAR topic names",
        ),

        # Left side: add a zero-filled 'time' field (required by FAST-LIO for
        # deskewing; gz gpu_lidar scans are instantaneous so the field is 0).
        Node(
            package="lidar_bridge",
            executable="add_time_field",
            name="add_time_field_left",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": ["/", model, "/scan_left/points"],
                "output_topic": ["/", model, "/scan_left/points_timed"],
            }],
        ),

        # Right side.
        Node(
            package="lidar_bridge",
            executable="add_time_field",
            name="add_time_field_right",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": ["/", model, "/scan_right/points"],
                "output_topic": ["/", model, "/scan_right/points_timed"],
            }],
        ),

        # Fuse both sides into one base_link cloud for FAST-LIO. Keeps the
        # same topics lidar_merge consumed from utils/start_fastlio.sh so the
        # pipeline behavior is unchanged.
        Node(
            package="lidar_bridge",
            executable="lidar_merge",
            name="lidar_merge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_left": ["/", model, "/scan_left/points"],
                "input_right": ["/", model, "/scan_right/points"],
                "output_topic": ["/", model, "/scan/points_fused"],
            }],
        ),
    ])
