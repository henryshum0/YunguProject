"""Coverage search skill composed from planner and navigation primitives."""

from __future__ import annotations

from nav_msgs.msg import Path
from rclpy.node import Node

from skills.base import Skill
from skills.navigate import NavigateSkill
from skills.primitives import PlanSearchPrimitive
from skills.primitives.plan_search import SearchArea


class SearchSkill(Skill[SearchArea, Path]):
    """Plan a four-corner search area, then queue its route for navigation."""

    def __init__(
        self,
        node: Node,
        *,
        frame_id: str = "map",
        service_name: str = "/coverage_planner/plan_coverage",
        goal_topic: str = "/waypoint_buffer",
    ) -> None:
        self._search = PlanSearchPrimitive(
            node, frame_id=frame_id, service_name=service_name)
        self._navigate = NavigateSkill(
            node, frame_id=frame_id, goal_topic=goal_topic)

    @property
    def name(self) -> str:
        return "search"

    @property
    def service_name(self) -> str:
        return self._search.service_name

    @property
    def goal_topic(self) -> str:
        return self._navigate.goal_topic

    def call(
        self,
        request: SearchArea,
        *,
        timeout_sec: float | None = 10.0,
    ) -> Path:
        """Plan a search area and queue its ENU path with ``NavigateSkill``."""
        path = self._search.call(request, timeout_sec=timeout_sec)
        self._navigate.call_poses(path.poses, timeout_sec=timeout_sec)
        return path
