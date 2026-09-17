"""Start everything the vehicle runs, in one launch.

The simulator is the environment; this is the vehicle. Sensor bridging,
navigation, coverage planning and detection are all parts of the same autonomy
stack, and running them as four separate terminals only made the operator keep
track of a startup order that the launch system can enforce itself.

So the normal session is two terminals:

    ./utils/start_sim.sh                             # Gazebo, PX4 SITL, bridges
    ros2 launch "$PWD/src/launch/yungu_stack.launch.py"
    python3 gui.py                                   # or a third for the GUI

or one, with ``./utils/start_all.sh``.

Nothing here has to start in a particular order: ROS subscriptions bind late, and
the target spawner waits for the Gazebo world to appear, so the whole stack can
come up while the simulator is still loading.

Each layer can be left out, which is how a partial stack is tested:

    ros2 launch "$PWD/src/launch/yungu_stack.launch.py" detection:=false
    ros2 launch "$PWD/src/launch/yungu_stack.launch.py" visualization:=true
    ros2 launch "$PWD/src/launch/yungu_stack.launch.py" mock_detector:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Create the combined vehicle-stack launch description."""
    here = Path(__file__).resolve().parent
    sensors_launch = Path(get_package_share_directory("gz_sensor_interface")) / "launch" \
        / "sensor_sensors.launch.py"
    navigation_launch = here / "coverage_and_offboard.launch.py"
    detection_launch = Path(get_package_share_directory("detection")) / "launch" \
        / "detection.launch.py"
    agents_launch = Path(get_package_share_directory("agents")) / "launch" \
        / "agents.launch.py"
    visualization_launch = Path(get_package_share_directory("visualization")) / "launch" \
        / "visualization.launch.py"
    default_planner_config = Path(get_package_share_directory("coverage_planner")) / "config" \
        / "yungu_planner.json"

    return LaunchDescription([
        DeclareLaunchArgument(
            "sensors", default_value="true",
            description="Bridge the Gazebo LiDAR, IMU and odometry into the navigation frames."),
        DeclareLaunchArgument(
            "navigation", default_value="true",
            description="Start the offboard FSM, SUPER, and the coverage-planning service."),
        DeclareLaunchArgument(
            "detection", default_value="true",
            description="Start the detection layer and spawn the simulated search targets."),
        DeclareLaunchArgument(
            "agents", default_value="true",
            description="Spawn the scripted ground robots and the node that walks them."),
        DeclareLaunchArgument(
            "visualization", default_value="false",
            description="Also start RViz and the birdview tools."),
        # Not "planner_config": offboard.launch.py already uses that name for the
        # SUPER trajectory-planner config, and launch configurations are inherited
        # by included descriptions, so the two would collide.
        DeclareLaunchArgument(
            "coverage_config", default_value=str(default_planner_config),
            description="Coverage planner schema-1.2 JSON used by the planning service."),
        DeclareLaunchArgument(
            "mock_detector", default_value="true",
            description="Run the simulation mock detector. Set false when a real detector runs."),
        DeclareLaunchArgument(
            "spawn_targets", default_value="true",
            description="Spawn the ground-truth targets into the Gazebo world."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sensors_launch)),
            condition=IfCondition(LaunchConfiguration("sensors")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation_launch)),
            condition=IfCondition(LaunchConfiguration("navigation")),
            launch_arguments={
                "config_file": LaunchConfiguration("coverage_config"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detection_launch)),
            condition=IfCondition(LaunchConfiguration("detection")),
            launch_arguments={
                "mock": LaunchConfiguration("mock_detector"),
                "spawn": LaunchConfiguration("spawn_targets"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(agents_launch)),
            condition=IfCondition(LaunchConfiguration("agents")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(visualization_launch)),
            condition=IfCondition(LaunchConfiguration("visualization")),
        ),
    ])
