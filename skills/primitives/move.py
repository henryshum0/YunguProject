"""Primitive that queues route goals with the offboard FSM."""

from __future__ import annotations

from collections.abc import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from offboard_fsm.srv import QueueWaypoints
from skills.base import Primitive, SkillExecutionError, SkillTimeoutError


class MovePrimitive(Primitive[Sequence[PoseStamped], int]):
    """Queue ENU route goals with an already-running ``offboard_fsm`` node."""

    def __init__(self, node: Node, *, queue_service: str = "/waypoint_buffer") -> None:
        self._node = node
        self._queue_service = queue_service
        self._client = node.create_client(QueueWaypoints, queue_service)

    @property
    def name(self) -> str:
        return "move"

    @property
    def queue_service(self) -> str:
        return self._queue_service

    def call(self, request: Sequence[PoseStamped], *, timeout_sec: float | None = 10.0) -> int:
        """Queue every supplied goal and return the accepted waypoint count."""
        goals = tuple(request)
        if not goals:
            raise ValueError("move requires at least one PoseStamped goal")
        for goal in goals:
            if not isinstance(goal, PoseStamped):
                raise TypeError("move goals must be geometry_msgs/msg/PoseStamped")
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise SkillTimeoutError(
                f"waypoint queue service '{self._queue_service}' is unavailable")
        service_request = QueueWaypoints.Request()
        service_request.waypoints = list(goals)
        future = self._client.call_async(service_request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            raise SkillTimeoutError(
                f"waypoint queue service '{self._queue_service}' did not respond")
        try:
            response = future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"waypoint queue service '{self._queue_service}' failed: {exc}") from exc
        if response is None:
            raise SkillExecutionError("waypoint queue service returned no response")
        if not response.success:
            raise SkillExecutionError(response.message or "waypoint queue rejected the route")
        if response.queued_count != len(goals):
            raise SkillExecutionError(
                f"waypoint queue accepted {response.queued_count} of {len(goals)} waypoints")
        return int(response.queued_count)
