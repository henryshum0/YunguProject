from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the offboard position controller with configurable parameters."""

    return LaunchDescription([
        DeclareLaunchArgument(
            'takeoff_height',
            default_value='2.5',
            description='Takeoff height in meters (positive = up)',
        ),
        DeclareLaunchArgument(
            'hover_x',
            default_value='0.0',
            description='Hover position X [m] (NED)',
        ),
        DeclareLaunchArgument(
            'hover_y',
            default_value='0.0',
            description='Hover position Y [m] (NED)',
        ),
        DeclareLaunchArgument(
            'hover_z',
            default_value='-2.5',
            description='Hover position Z [m] (NED, negative = up)',
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='50.0',
            description='Setpoint stream rate [Hz]',
        ),

        Node(
            package='offboard_control',
            executable='position_controller',
            name='position_controller',
            output='screen',
            parameters=[{
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'hover_x': LaunchConfiguration('hover_x'),
                'hover_y': LaunchConfiguration('hover_y'),
                'hover_z': LaunchConfiguration('hover_z'),
                'update_rate': LaunchConfiguration('update_rate'),
            }],
        ),
    ])
