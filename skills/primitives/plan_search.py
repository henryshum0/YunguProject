"""Primitive that requests coverage waypoints from coverage_planner."""

from __future__ import annotations

from collections.abc import Sequence
from math import isfinite

import rclpy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node

from coverage_planner.action import PlanCoverage
from skills.base import Primitive, SkillExecutionError, SkillTimeoutError
from skills.config import SkillRuntimeConfig


SearchArea = Sequence[tuple[float, float]]


class PlanSearchPrimitive(Primitive[SearchArea, Path]):
    """Request a coverage route for a four-corner search area in the map frame."""

    def __init__(
        self,
        node: Node,
        *,
        config: SkillRuntimeConfig,
    ) -> None:
        self._node = node
        self._frame_id = config.coverage_planner.frame_id
        self._service_name = config.coverage_planner.plan_service
        self._client = ActionClient(node, PlanCoverage, self._service_name)

    @property
    def name(self) -> str:
        return "plan_search"

    @property
    def service_name(self) -> str:
        return self._service_name

    def call(
        self,
        request: SearchArea,
        *,
        publish_result: bool = False,
        timeout_sec: float | None = 30.0,
    ) -> Path:
        """Submit a planner action and return its asynchronous sparse waypoint result.

        ``timeout_sec`` covers server discovery and receipt of the action goal.
        Once accepted, the planner result waits asynchronously without the GUI's
        short request timeout. ``publish_result=False`` is a dry run: the route
        is returned only in the action result.
        """
        corners = _validated_corners(request)
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise SkillTimeoutError(
                f"coverage planner action '{self._service_name}' is unavailable")
        goal = PlanCoverage.Goal()
        goal.search_area.header.frame_id = self._frame_id
        goal.search_area.polygon.points = [
            Point32(x=x, y=y, z=0.0) for x, y in corners
        ]
        goal.publish_result = bool(publish_result)
        receipt_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, receipt_future, timeout_sec=timeout_sec)
        if not receipt_future.done():
            receipt_future.cancel()
            raise SkillTimeoutError(
                f"coverage planner action '{self._service_name}' did not acknowledge the request")
        try:
            goal_handle = receipt_future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"coverage planner action '{self._service_name}' receipt failed: {exc}"
            ) from exc
        if goal_handle is None or not goal_handle.accepted:
            raise SkillExecutionError("coverage planner rejected the request")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=None)
        if not result_future.done():  # defensive: a shutdown can end the spin without a result
            raise SkillTimeoutError(
                f"coverage planner action '{self._service_name}' ended before returning a result")
        try:
            action_result = result_future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"coverage planner action '{self._service_name}' failed: {exc}"
            ) from exc
        if action_result is None or action_result.result is None:
            raise SkillExecutionError("coverage planner action returned no result")
        response = action_result.result
        if not response.success:
            raise SkillExecutionError(response.message or "coverage planning failed")
        return response.waypoints


def _validated_corners(request: SearchArea) -> tuple[tuple[float, float], ...]:
    corners = tuple((float(x), float(y)) for x, y in request)
    if len(corners) != 4:
        raise ValueError(f"search requires exactly four corners, got {len(corners)}")
    if len(set(corners)) != 4:
        raise ValueError("search corners must be distinct")
    if not all(isfinite(value) for point in corners for value in point):
        raise ValueError("search corners must be finite")
    return corners
