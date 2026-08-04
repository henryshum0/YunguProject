import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
        DeclareLaunchArgument('update_rate', default_value='50.0'),
        DeclareLaunchArgument('planner_cmd_hz', default_value='10.0',
                              description='Cmd rate threshold for planner hand-over [Hz]'),
        DeclareLaunchArgument('planner_enter_delay', default_value='0.5'),
        DeclareLaunchArgument('planner_exit_delay', default_value='1.0'),
        DeclareLaunchArgument('arm_wait', default_value='2.0'),
        DeclareLaunchArgument('offboard_wait', default_value='3.0'),
        DeclareLaunchArgument('takeoff_height', default_value='1.5',
                              description='NED takeoff height (negative = up) [m]'),
        DeclareLaunchArgument('takeoff_vel', default_value='1.0'),
        DeclareLaunchArgument('landing_vel', default_value='0.5'),
        DeclareLaunchArgument('landing_z', default_value='0.15'),
        DeclareLaunchArgument('cmd_topic', default_value='/planning/pos_cmd'),
        DeclareLaunchArgument('local_pos_topic',
                              default_value='/fmu/out/vehicle_local_position_v1',
                              description='PX4 local position topic (note the _v1 suffix)'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='RViz 2D Goal Pose topic'),
        DeclareLaunchArgument('goal_marker_topic', default_value='/goal_marker',
                              description='Goal visualization marker topic'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 visualization'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('planner_config', default_value='gazebo.yaml',
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
                'offboard_wait': LaunchConfiguration('offboard_wait'),
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'takeoff_vel': LaunchConfiguration('takeoff_vel'),
                'landing_vel': LaunchConfiguration('landing_vel'),
                'landing_z': LaunchConfiguration('landing_z'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'local_pos_topic': LaunchConfiguration('local_pos_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'goal_marker_topic': LaunchConfiguration('goal_marker_topic'),
            }],
        ),

        # ------------------------------------------------------------------
        # SUPER planner (fsm_node)
        #   - loads config from super_planner/config/<planner_config>
        #   - subscribes to /x500_lidar/scan/points + /odom (gz bridge)
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
