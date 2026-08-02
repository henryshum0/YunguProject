from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the SUPER→PX4 bridge with configurable parameters."""

    return LaunchDescription([
        DeclareLaunchArgument(
            'update_rate',
            default_value='50.0',
            description='Setpoint stream rate to PX4 [Hz]',
        ),
        DeclareLaunchArgument(
            'offboard_timeout',
            default_value='5.0',
            description='Timeout before holding position if SUPER stops [s]',
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/planning/pos_cmd',
            description='SUPER PositionCommand topic to subscribe to',
        ),

        Node(
            package='super_px4_bridge',
            executable='super_px4_bridge_node',
            name='super_px4_bridge',
            output='screen',
            parameters=[{
                'update_rate': LaunchConfiguration('update_rate'),
                'offboard_timeout': LaunchConfiguration('offboard_timeout'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
            }],
        ),
    ])
