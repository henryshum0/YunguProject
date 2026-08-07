from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='Goal click topic (geometry_msgs/PoseStamped)'),
        DeclareLaunchArgument('cmd_topic', default_value='/planning/pos_cmd',
                              description='SUPER command trajectory topic'),
        DeclareLaunchArgument('odom_topic', default_value='/lidar_slam/odom',
                              description='Real drone odometry topic (nav_msgs/Odometry)'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='Log directory (empty = <project>/cmd_log)'),
        DeclareLaunchArgument('min_cmd_rate', default_value='10.0',
                              description='Stop recording when cmd rate drops below this [Hz]'),
        DeclareLaunchArgument('viz_en', default_value='true',
                              description='Enable live sliding-window matplotlib view'),
        DeclareLaunchArgument('window_sec', default_value='20.0',
                              description='Sliding window length [s] for the live view'),
        DeclareLaunchArgument('plot_rate', default_value='10.0',
                              description='Live plot refresh rate [Hz]'),

        Node(
            package='cmd_record',
            executable='cmd_record_node',
            name='cmd_record',
            output='screen',
            parameters=[{
                'goal_topic': LaunchConfiguration('goal_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_dir': LaunchConfiguration('log_dir'),
                'min_cmd_rate': LaunchConfiguration('min_cmd_rate'),
                'viz_en': LaunchConfiguration('viz_en'),
                'window_sec': LaunchConfiguration('window_sec'),
                'plot_rate': LaunchConfiguration('plot_rate'),
            }],
        ),
    ])
