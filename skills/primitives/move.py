"""Primitive that sends route goals to the offboard FSM."""

from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from skills.base import Primitive


class MovePrimitive(Primitive[Sequence[PoseStamped], int]):
    """Publish ENU route goals to an already-running ``offboard_fsm`` node."""

    def __init__(self, node: Node, *, goal_topic: str = "/waypoint_buffer") -> None:
        self._goal_topic = goal_topic
        self._publisher = node.create_publisher(
            PoseStamped,
            goal_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    @property
    def name(self) -> str:
        return "move"

    @property
    def goal_topic(self) -> str:
        return self._goal_topic

    def call(self, request: Sequence[PoseStamped], *, timeout_sec: float | None = None) -> int:
        """Publish every supplied goal and return the count of sent waypoints."""
        del timeout_sec
        goals = tuple(request)
        if not goals:
            raise ValueError("move requires at least one PoseStamped goal")
        for goal in goals:
            if not isinstance(goal, PoseStamped):
                raise TypeError("move goals must be geometry_msgs/msg/PoseStamped")
            self._publisher.publish(goal)
        return len(goals)
