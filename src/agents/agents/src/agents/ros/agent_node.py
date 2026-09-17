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
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from agents.config import AgentSpec, AgentsConfig
from agents.gait import Gait
from agents.navigator import Navigator
from agents.topics import SIM_CLOCK_TOPIC, base_command_topic, joint_command_topic

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
        self._nominal_dt = 1.0 / rate_hz
        #: Latest simulated time, and the value it had on the previous tick.
        self._sim_time: float | None = None
        self._last_sim_time: float | None = None
        self._warned_about_the_clock = False
        # Depth 1, best effort: only the newest reading matters, and a late one
        # is worse than none.
        self.create_subscription(
            Clock, SIM_CLOCK_TOPIC, self._on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
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

    def _on_clock(self, message: Clock) -> None:
        self._sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def _elapsed(self) -> float | None:
        """Simulated seconds since the last tick, or ``None`` to skip this one.

        The agents must be dead-reckoned on the clock Gazebo moves them with, not
        on the wall clock. Gazebo applies a commanded velocity over *simulated*
        time, so whenever the real-time factor is below 1 -- which it is as soon
        as the GUI, PX4 and the cameras are all running -- a tick of wall clock
        buys less motion than a tick of sim time. Integrating 1/rate_hz per tick
        therefore credits the agent with distance and rotation it never made. The
        error is a fixed fraction of every movement and nothing ever corrects it,
        so a goal behind the robot came out as a walk off at an angle.

        Returning ``None`` leaves the previous command latched in Gazebo, which
        is what we want: the next tick sees the whole interval and integrates it
        in one go, so no motion is lost. The interval is deliberately not capped
        -- if this node is starved for a moment, Gazebo really did keep moving
        for all of it, and dead reckoning has to account for every bit.
        """
        if self._sim_time is None:
            if not self._warned_about_the_clock:
                self._warned_about_the_clock = True
                self.get_logger().warn(
                    f"no simulated clock on {SIM_CLOCK_TOPIC}: falling back to the wall "
                    "clock. Agents will drift from where they are drawn whenever the "
                    "simulation runs slower than real time. Is the agents bridge up?")
            return self._nominal_dt
        if self._last_sim_time is None:
            self._last_sim_time = self._sim_time
            return None
        dt = self._sim_time - self._last_sim_time
        self._last_sim_time = self._sim_time
        # Zero while paused; negative if the simulation was reset under us.
        return dt if dt > 0.0 else None

    def _tick(self) -> None:
        dt = self._elapsed()
        if dt is None:
            return
        for runtime in self._runtimes:
            step = runtime.navigator.update(dt)

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
