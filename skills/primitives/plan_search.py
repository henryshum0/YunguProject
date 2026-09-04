"""Primitive that requests coverage waypoints from coverage_planner."""

from __future__ import annotations

from collections.abc import Sequence
from math import isfinite

import rclpy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from rclpy.node import Node

from coverage_planner.srv import PlanCoverage
from skills.base import Primitive, SkillExecutionError, SkillTimeoutError


SearchArea = Sequence[tuple[float, float]]


class PlanSearchPrimitive(Primitive[SearchArea, Path]):
    """Request a coverage route for a four-corner search area in the map frame."""

    def __init__(
        self,
        node: Node,
        *,
        frame_id: str = "map",
        service_name: str = "/coverage_planner/plan_coverage",
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._service_name = service_name
        self._client = node.create_client(PlanCoverage, service_name)

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
        """Wait for the planner service, then return its sparse waypoint path.

        ``publish_result=False`` is a dry run: the route is returned only in the
        service response. Set it when a planner `Path`/marker visualization is
        desired as well.
        """
        corners = _validated_corners(request)
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise SkillTimeoutError(
                f"coverage planner service '{self._service_name}' is unavailable")
        service_request = PlanCoverage.Request()
        service_request.search_area.header.frame_id = self._frame_id
        service_request.search_area.polygon.points = [
            Point32(x=x, y=y, z=0.0) for x, y in corners
        ]
        service_request.publish_result = bool(publish_result)
        future = self._client.call_async(service_request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            raise SkillTimeoutError(
                f"coverage planner service '{self._service_name}' did not respond")
        try:
            response = future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"coverage planner service '{self._service_name}' failed: {exc}"
            ) from exc
        if response is None:
            raise SkillExecutionError("coverage planner service returned no response")
        if not response.success:
            raise SkillExecutionError(response.message or "coverage planner rejected the search area")
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
