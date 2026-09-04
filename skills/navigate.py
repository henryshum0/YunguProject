"""High-level coordinate navigation skill backed by ``MovePrimitive``."""

from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from skills.base import Skill
from skills.frames import WaypointInput, pose_stamped_from_enu_waypoint, to_enu_waypoints
from skills.primitives import ClearWaypointsPrimitive, MovePrimitive


class NavigateSkill(Skill[WaypointInput, int]):
    """Send one or more ENU or NED coordinate waypoints to the offboard FSM."""

    def __init__(
        self,
        node: Node,
        *,
        frame_id: str = "map",
        queue_service: str = "/waypoint_buffer",
        clear_service: str = "/waypoint_buffer/clear",
    ) -> None:
        self._frame_id = frame_id
        self._move = MovePrimitive(node, queue_service=queue_service)
        self._clear = ClearWaypointsPrimitive(node, clear_service=clear_service)

    @property
    def name(self) -> str:
        return "navigate"

    @property
    def frame_id(self) -> str:
        return self._frame_id

    @property
    def queue_service(self) -> str:
        return self._move.queue_service

    @property
    def clear_service(self) -> str:
        return self._clear.clear_service

    def call(
        self,
        request: WaypointInput,
        *,
        frame: str = "enu",
        timeout_sec: float | None = 10.0,
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
        timeout_sec: float | None = 10.0,
    ) -> int:
        """Queue pre-built ENU poses through the configured move primitive."""
        return self._move.call(poses, timeout_sec=timeout_sec)

    def clear(self, *, timeout_sec: float | None = 10.0) -> int:
        """Abort the active route and clear all queued waypoints."""
        return self._clear.call(timeout_sec=timeout_sec)
