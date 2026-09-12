"""Coverage search skill composed from planner and navigation primitives."""

from __future__ import annotations

from nav_msgs.msg import Path
from rclpy.node import Node

from skills.config import SkillRuntimeConfig
from skills.primitives import PlanSearchPrimitive
from skills.primitives.plan_search import SearchArea
from skills.skills.base import Skill
from skills.skills.navigate import NavigateSkill


class SearchSkill(Skill[SearchArea, Path]):
    """Plan a four-corner search area, then queue its route for navigation."""

    def __init__(self, node: Node, *, config: SkillRuntimeConfig) -> None:
        self._search = PlanSearchPrimitive(node, config=config)
        self._navigate = NavigateSkill(node, config=config)

    @property
    def name(self) -> str:
        return "search"

    @property
    def service_name(self) -> str:
        return self._search.service_name

    @property
    def queue_service(self) -> str:
        return self._navigate.queue_service

    def call(
        self,
        request: SearchArea,
        *,
        timeout_sec: float | None = 30.0,
    ) -> Path:
        """Plan a search area and queue its ENU path with ``NavigateSkill``."""
        path = self._search.call(request, publish_result=True, timeout_sec=timeout_sec)
        self._navigate.call_poses(path.poses, timeout_sec=timeout_sec)
        return path
