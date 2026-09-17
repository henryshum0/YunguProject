"""Spawn every configured ground agent and start the node that drives them.

Three things have to line up for an agent to move, and all three are derived from
the same ``agents.yaml`` so they cannot drift apart:

* the model is spawned at the agent's ENU spawn pose, converted to Gazebo world
  coordinates;
* the driver node is started with that config;
* a bridge is created for exactly the topics that agent's model listens on — the
  base velocity, plus one per animated joint.

The bridge list is generated rather than written out because it is long and
entirely mechanical: two agents with eight animated joints each is eighteen
topics, and a hand-maintained list would silently lose a leg the first time a
gait gained a joint.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _agent_actions(context, *_args, **_kwargs):
    """Build the spawn, bridge and driver actions from the resolved config."""
    # Imported here, not at module scope: the package's Python tree is only on
    # PYTHONPATH once the workspace overlay is sourced, which is true when launch
    # runs this but not necessarily when it first imports the file.
    from agents.config import AgentsConfig
    from agents.topics import (
        GZ_CLOCK_TOPIC, SIM_CLOCK_TOPIC, base_command_topic, joint_command_topic)

    config_file = LaunchConfiguration("agents_config").perform(context)
    models_dir = Path(LaunchConfiguration("models_dir").perform(context))
    config = AgentsConfig.load(config_file)

    actions: list = []
    # The agent models are built from meshes referenced as `model://`, which
    # Gazebo resolves through its resource path and *not* relative to the file
    # they are spawned from. When that path is missing the meshes silently fail
    # to load — the robot spawns and walks, but renders as only the primitive
    # shapes it contains — so say so loudly rather than leaving it to be noticed
    # as a half-drawn robot. utils/start_sim.sh sets this before Gazebo starts.
    resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if str(models_dir) not in resource_path.split(os.pathsep):
        actions.append(LogInfo(msg=(
            f"WARNING: '{models_dir}' is not on GZ_SIM_RESOURCE_PATH. Agent meshes will not "
            "render; the robots will appear as a few loose primitive shapes. Start the "
            "simulator with utils/start_sim.sh, or export that path before launching Gazebo.")))
    bridge_arguments: list[str] = []
    camera_topics: list[str] = []

    for spec in config.agents:
        model_file = models_dir / spec.model / "model.sdf"
        x, y, z = config.gz_spawn_pose(spec)
        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{spec.name}",
            output="screen",
            arguments=[
                "-file", str(model_file),
                "-name", spec.model,
                "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", f"{z:.4f}",
                "-Y", f"{spec.spawn.yaw:.6f}",
            ],
        ))
        # ROS -> Gazebo only: nothing reads these back, the driver already knows
        # what it commanded.
        bridge_arguments.append(
            f"{base_command_topic(spec.model)}@geometry_msgs/msg/Twist]gz.msgs.Twist")
        for joint in spec.gait.joint_names:
            bridge_arguments.append(
                f"{joint_command_topic(spec.model, joint)}@std_msgs/msg/Float64]gz.msgs.Double")
        if spec.camera is not None:
            camera_topics.append(spec.camera.topic)

    # Gazebo -> ROS, and the one thing the driver reads back. It integrates its
    # dead reckoning on simulated time, because that is the clock Gazebo moves
    # the robots on: whenever the real-time factor is not exactly 1 the wall
    # clock runs faster, and an agent that dead-reckons on it believes it has
    # walked further and turned further than it has.
    bridge_arguments.append(f"{GZ_CLOCK_TOPIC}@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="agents_bridge",
        output="screen",
        arguments=bridge_arguments,
        remappings=[(GZ_CLOCK_TOPIC, SIM_CLOCK_TOPIC)],
    ))
    if camera_topics:
        # Images go through the dedicated image bridge rather than the parameter
        # bridge: it is the one the workspace already uses for the UAV feeds.
        actions.append(Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="agents_camera_bridge",
            output="screen",
            arguments=camera_topics,
        ))
    actions.append(Node(
        package="agents",
        executable="agent_node",
        name="agent_driver",
        output="screen",
        arguments=["--config", config_file],
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    """Create the ground-agent launch description."""
    share = Path(get_package_share_directory("agents"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "agents_config", default_value=str(share / "config" / "agents.yaml"),
            description="Agents YAML: which robots exist, where they start, how they walk."),
        DeclareLaunchArgument(
            "models_dir", default_value=str(share / "models"),
            description="Directory holding the agent Gazebo models."),
        OpaqueFunction(function=_agent_actions),
    ])
