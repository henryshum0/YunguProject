"""The node that drives every configured ground agent.

One node runs all agents rather than one node per agent: they share a clock and a
config file, and a single timer keeps their gaits in step with their motion
without any cross-process coordination.

Each tick per agent is the whole loop:

    goal -> navigator -> body velocity -> Gazebo
                      -> distance covered -> gait -> joint angles -> Gazebo

An idle agent is commanded to a stop and held at its stance pose, so a parked
robot stands rather than freezing mid-stride.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from agents.config import AgentSpec, AgentsConfig
from agents.gait import Gait
from agents.navigator import Navigator
from agents.topics import base_command_topic, joint_command_topic

#: Control rate. Fast enough that the legs look continuous and the dead-reckoned
#: pose stays close to what the simulator integrates from the same velocities.
DEFAULT_RATE_HZ = 50.0


@dataclass
class _Runtime:
    """Everything one agent needs while running."""

    spec: AgentSpec
    navigator: Navigator
    gait: Gait
    cmd_vel: object
    joint_publishers: dict[str, object]
    pose_publisher: object
    phase: float = 0.0
    #: Set while the agent is walking, so the arrival message is logged once.
    was_walking: bool = False


class AgentDriverNode(Node):
    """Drives every configured agent: goals in, velocity and joint angles out."""

    def __init__(self, config: AgentsConfig, *, rate_hz: float = DEFAULT_RATE_HZ) -> None:
        super().__init__("agent_driver")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self._config = config
        self._runtimes: list[_Runtime] = [self._build(spec) for spec in config.agents]
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self._dt = 1.0 / rate_hz
        self.get_logger().info(
            f"driving {len(self._runtimes)} agent(s): "
            f"{', '.join(runtime.spec.name for runtime in self._runtimes)}")

    def _build(self, spec: AgentSpec) -> _Runtime:
        # Commands are state, not a stream: keep the last one for a late-joining
        # subscriber and never drop one, or an agent can miss its stop command.
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        runtime = _Runtime(
            spec=spec,
            navigator=Navigator(spec.spawn, limits=spec.limits),
            gait=spec.gait,
            cmd_vel=self.create_publisher(
                Twist, base_command_topic(spec.model), reliable),
            joint_publishers={
                joint: self.create_publisher(
                    Float64, joint_command_topic(spec.model, joint), reliable)
                for joint in spec.gait.joint_names
            },
            pose_publisher=self.create_publisher(PoseStamped, spec.pose_topic, reliable),
        )
        self.create_subscription(
            PoseStamped, spec.goal_topic,
            lambda message, name=spec.name: self._on_goal(name, message), reliable)
        self.get_logger().info(
            f"agent '{spec.name}' ({spec.model}): goals on {spec.goal_topic}, "
            f"{len(runtime.joint_publishers)} animated joint(s)")
        return runtime

    def _on_goal(self, name: str, message: PoseStamped) -> None:
        runtime = self._runtime(name)
        if runtime is None:
            return
        goal_x = message.pose.position.x
        goal_y = message.pose.position.y
        runtime.navigator.set_goal(goal_x, goal_y)
        self.get_logger().info(
            f"agent '{name}': walking to ({goal_x:.2f}, {goal_y:.2f})")

    def _runtime(self, name: str) -> _Runtime | None:
        for runtime in self._runtimes:
            if runtime.spec.name == name:
                return runtime
        return None

    def _tick(self) -> None:
        for runtime in self._runtimes:
            step = runtime.navigator.update(self._dt)

            command = Twist()
            command.linear.x = step.command.linear_x
            command.angular.z = step.command.angular_z
            runtime.cmd_vel.publish(command)

            if step.distance_m > 0.0:
                runtime.phase = runtime.gait.advance(runtime.phase, step.distance_m)
                angles = runtime.gait.positions(runtime.phase)
                runtime.was_walking = True
            elif runtime.was_walking:
                # Just stopped: settle onto the stance pose instead of holding
                # whatever half-step the agent happened to end on.
                angles = runtime.gait.stance()
                runtime.was_walking = False
                self.get_logger().info(f"agent '{runtime.spec.name}': arrived, holding position")
            else:
                angles = runtime.gait.stance()

            for joint, angle in angles.items():
                publisher = runtime.joint_publishers.get(joint)
                if publisher is not None:
                    publisher.publish(Float64(data=float(angle)))

            self._publish_pose(runtime, step.pose)

    def _publish_pose(self, runtime: _Runtime, pose) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        # Built from the floor, like the spawn height: the ENU origin is the
        # drone's launch point, not the ground.
        message.pose.position.z = self._config.ground_z_m + runtime.spec.base_height_m
        message.pose.orientation.z = sin(pose.yaw / 2.0)
        message.pose.orientation.w = cos(pose.yaw / 2.0)
        runtime.pose_publisher.publish(message)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="path to the agents YAML")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ,
                        help="control and animation rate")
    arguments, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    config = AgentsConfig.load(arguments.config)
    rclpy.init(args=ros_args)
    node = AgentDriverNode(config, rate_hz=arguments.rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
