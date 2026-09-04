"""Primitive that clears the offboard FSM waypoint queue."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from offboard_fsm.srv import ClearWaypoints
from skills.base import Primitive, SkillExecutionError, SkillTimeoutError


class ClearWaypointsPrimitive(Primitive[None, int]):
    """Abort the active waypoint and clear all queued offboard waypoints."""

    def __init__(self, node: Node, *, clear_service: str = "/waypoint_buffer/clear") -> None:
        self._node = node
        self._clear_service = clear_service
        self._client = node.create_client(ClearWaypoints, clear_service)

    @property
    def name(self) -> str:
        return "clear_waypoints"

    @property
    def clear_service(self) -> str:
        return self._clear_service

    def call(self, request: None = None, *, timeout_sec: float | None = 10.0) -> int:
        """Clear the route and return the active-plus-queued waypoint count removed."""
        if request is not None:
            raise ValueError("clear_waypoints does not accept a request payload")
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise SkillTimeoutError(
                f"waypoint clear service '{self._clear_service}' is unavailable")
        future = self._client.call_async(ClearWaypoints.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            raise SkillTimeoutError(
                f"waypoint clear service '{self._clear_service}' did not respond")
        try:
            response = future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"waypoint clear service '{self._clear_service}' failed: {exc}") from exc
        if response is None:
            raise SkillExecutionError("waypoint clear service returned no response")
        if not response.success:
            raise SkillExecutionError(response.message or "waypoint clear request was rejected")
        return int(response.cleared_count)
