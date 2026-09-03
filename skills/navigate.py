"""High-level coordinate navigation skill backed by ``MovePrimitive``."""

from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from skills.base import Skill
from skills.frames import WaypointInput, pose_stamped_from_enu_waypoint, to_enu_waypoints
from skills.primitives import MovePrimitive


class NavigateSkill(Skill[WaypointInput, int]):
    """Send one or more ENU or NED coordinate waypoints to the offboard FSM."""

    def __init__(
        self,
        node: Node,
        *,
        frame_id: str = "map",
        goal_topic: str = "/waypoint_buffer",
    ) -> None:
        self._frame_id = frame_id
        self._move = MovePrimitive(node, goal_topic=goal_topic)

    @property
    def name(self) -> str:
        return "navigate"

    @property
    def frame_id(self) -> str:
        return self._frame_id

    @property
    def goal_topic(self) -> str:
        return self._move.goal_topic

    def call(
        self,
        request: WaypointInput,
        *,
        frame: str = "enu",
        timeout_sec: float | None = None,
    ) -> int:
        """Convert frame-native coordinates and publish them through ``MovePrimitive``."""
        poses = tuple(
            pose_stamped_from_enu_waypoint(waypoint, frame_id=self._frame_id)
            for waypoint in to_enu_waypoints(request, frame=frame)
        )
        return self.call_poses(poses, timeout_sec=timeout_sec)

    def call_poses(
        self,
        poses: Sequence[PoseStamped],
        *,
        timeout_sec: float | None = None,
    ) -> int:
        """Publish pre-built ENU poses through the configured move primitive."""
        return self._move.call(poses, timeout_sec=timeout_sec)
