"""Threaded ROS subscriptions for the persistent operations map."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from functools import partial
from math import atan2, degrees, isfinite
from queue import Empty, Full, Queue
from threading import Event, Thread
from time import monotonic
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data


Point = tuple[float, float]


@dataclass(frozen=True, slots=True)
class VehicleState:
    """Latest ENU vehicle pose received by the operations-map subscriber."""

    x: float
    y: float
    z: float
    heading_deg: float
    received_at: float


@dataclass(frozen=True, slots=True)
class QueueState:
    """Authoritative offboard route: active point first, then pending points."""

    points: tuple[Point, ...]
    received_at: float


@dataclass(frozen=True, slots=True)
class AgentState:
    """Latest ENU pose reported by one ground agent.

    Agents dead-reckon their own pose and publish it, so this is where they
    believe they are rather than a measurement. For a kinematic agent those are
    the same thing.
    """

    name: str
    x: float
    y: float
    heading_deg: float
    received_at: float


def agent_state_from_pose(name: str, message: Any, *, received_at: float | None = None) -> AgentState:
    """Validate a ground-agent pose and convert its orientation to ENU yaw."""
    pose = message.pose
    position = pose.position
    orientation = pose.orientation
    values = (position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(isfinite(float(value)) for value in values):
        raise ValueError(f"agent '{name}' pose contains non-finite values")
    yaw = atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return AgentState(
        name=name,
        x=float(position.x),
        y=float(position.y),
        heading_deg=degrees(yaw) % 360.0,
        received_at=monotonic() if received_at is None else received_at,
    )


def vehicle_state_from_odometry(message: Any, *, received_at: float | None = None) -> VehicleState:
    """Validate an ENU odometry message and convert its orientation to ENU yaw."""
    pose = message.pose.pose
    position = pose.position
    orientation = pose.orientation
    values = (position.x, position.y, position.z, orientation.x, orientation.y,
              orientation.z, orientation.w)
    if not all(isfinite(float(value)) for value in values):
        raise ValueError("vehicle odometry contains non-finite pose values")
    yaw = atan2(
        2.0 * (float(orientation.w) * float(orientation.z) +
               float(orientation.x) * float(orientation.y)),
        1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
    )
    return VehicleState(
        x=float(position.x), y=float(position.y), z=float(position.z),
        heading_deg=degrees(yaw) % 360.0,
        received_at=monotonic() if received_at is None else received_at,
    )


def queue_state_from_path(message: Any, *, received_at: float | None = None) -> QueueState:
    """Extract finite ENU x/y points from an offboard queue-status Path."""
    points: list[Point] = []
    for index, pose_stamped in enumerate(message.poses):
        position = pose_stamped.pose.position
        x, y = float(position.x), float(position.y)
        if not isfinite(x) or not isfinite(y):
            raise ValueError(f"queue status waypoint {index + 1} has non-finite x/y")
        points.append((x, y))
    return QueueState(tuple(points), monotonic() if received_at is None else received_at)


class OperationsTelemetry:
    """Own subscriptions/executor so telemetry cannot interfere with ROS services."""

    def __init__(
        self,
        odometry_topic: str,
        queue_status_topic: str,
        agent_pose_topics: Mapping[str, str] | None = None,
    ) -> None:
        self.odometry_topic = _topic(odometry_topic, "vehicle odometry")
        self.queue_status_topic = _topic(queue_status_topic, "waypoint queue status")
        #: Ground agents share this subscriber's node and thread: they are part
        #: of the same map picture, and one executor is cheaper than one each.
        self.agent_pose_topics = dict(agent_pose_topics or {})
        self._vehicles: Queue[VehicleState] = Queue(maxsize=1)
        self._queues: Queue[QueueState] = Queue(maxsize=1)
        self._agents: dict[str, Queue[AgentState]] = {}
        self._errors: Queue[str] = Queue(maxsize=1)
        self._stop = Event()
        self._node: Node = rclpy.create_node("skills_test_gui_telemetry")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._odom_subscription = self._node.create_subscription(
            Odometry, self.odometry_topic, self._on_odometry, qos_profile_sensor_data)
        queue_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._queue_subscription = self._node.create_subscription(
            Path, self.queue_status_topic, self._on_queue, queue_qos)
        self._agent_subscriptions = []
        for name, topic in self.agent_pose_topics.items():
            self._agents[name] = Queue(maxsize=1)
            self._agent_subscriptions.append(self._node.create_subscription(
                PoseStamped, _topic(topic, f"agent '{name}' pose"),
                partial(self._on_agent_pose, name), qos_profile_sensor_data))
        self._thread = Thread(target=self._spin, name="skills-operations-telemetry", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    @staticmethod
    def _put_latest(queue: Queue, value: object) -> None:
        try:
            queue.put_nowait(value)
            return
        except Full:
            pass
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(value)

    def _on_odometry(self, message: Odometry) -> None:
        try:
            self._put_latest(self._vehicles, vehicle_state_from_odometry(message))
        except ValueError as error:
            self._put_latest(self._errors, str(error))

    def _on_queue(self, message: Path) -> None:
        try:
            self._put_latest(self._queues, queue_state_from_path(message))
        except ValueError as error:
            self._put_latest(self._errors, str(error))

    def _on_agent_pose(self, name: str, message: PoseStamped) -> None:
        try:
            self._put_latest(self._agents[name], agent_state_from_pose(name, message))
        except ValueError as error:
            self._put_latest(self._errors, str(error))

    @staticmethod
    def _latest(queue: Queue):
        value = None
        while True:
            try:
                value = queue.get_nowait()
            except Empty:
                return value

    def latest_vehicle(self) -> VehicleState | None:
        return self._latest(self._vehicles)

    def latest_agents(self) -> dict[str, AgentState]:
        """Every ground agent that has reported since the last call.

        Agents that have not published again are absent rather than stale, so
        the caller keeps showing the pose it already has.
        """
        updates = {}
        for name, queue in self._agents.items():
            state = self._latest(queue)
            if state is not None:
                updates[name] = state
        return updates

    def latest_queue(self) -> QueueState | None:
        return self._latest(self._queues)

    def latest_error(self) -> str | None:
        return self._latest(self._errors)

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=1.0)
        self._node.destroy_node()


def _topic(value: str, label: str) -> str:
    topic = value.strip()
    if not topic:
        raise ValueError(f"{label} topic must not be empty")
    return topic
