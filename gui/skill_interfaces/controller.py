"""ROS-facing controller used by the standalone skills test GUI."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from collections.abc import Sequence

from skills import (
    LandPrimitive,
    NavigateSkill,
    PlanSearchPrimitive,
    SearchProgress,
    SearchResult,
    SearchSkill,
    SkillRuntimeConfig,
    TakeoffPrimitive,
)


@dataclass(frozen=True, slots=True)
class ConnectionSettings:
    config: SkillRuntimeConfig
    timeout_sec: float


class SkillController:
    """Create configured skills and call the offboard FSM command services."""

    def __init__(self, node: Node) -> None:
        self._node = node

    def takeoff(self, settings: ConnectionSettings) -> str:
        return TakeoffPrimitive(self._node, config=settings.config).call(
            timeout_sec=settings.timeout_sec)

    def land(self, settings: ConnectionSettings) -> str:
        return LandPrimitive(self._node, config=settings.config).call(
            timeout_sec=settings.timeout_sec)

    def navigate(
        self, waypoints: tuple[tuple[float, float, float, float], ...], *, frame: str,
        settings: ConnectionSettings,
    ) -> int:
        return NavigateSkill(self._node, config=settings.config).call(
            waypoints, frame=frame, timeout_sec=settings.timeout_sec)

    def clear(self, settings: ConnectionSettings) -> int:
        return NavigateSkill(self._node, config=settings.config).clear(timeout_sec=settings.timeout_sec)

    def plan_search(
        self, corners: tuple[tuple[float, float], ...], *, settings: ConnectionSettings,
    ) -> Path:
        return PlanSearchPrimitive(self._node, config=settings.config).call(
            corners,
            publish_result=False,
            timeout_sec=settings.timeout_sec,
        )

    def search_and_queue(
        self, corners: tuple[tuple[float, float], ...], *, settings: ConnectionSettings,
    ) -> Path:
        return SearchSkill(self._node, config=settings.config).call(
            corners, timeout_sec=settings.timeout_sec)

def format_search_result(result: SearchResult, truth_matches: Sequence | None = None) -> str:
    """Render a finished mission for the result panel.

    Target positions are what the mission *estimated*: each detection box
    back-projected onto the ground plane and averaged over frames. When
    ``truth_matches`` is supplied — simulation only, from
    ``detection.truth.match_to_truth`` — the true position is printed underneath
    so the error can be read directly instead of worked out by hand.
    """
    lines = [
        result.message,
        "",
        f"success            : {result.success}",
        f"found              : {result.found}",
        f"termination_reason : {result.termination_reason}",
        f"route progress     : {result.waypoints_completed}/{result.waypoints_total} "
        f"({result.route_completion * 100:.0f}%)",
        f"detector frames    : {result.detector_frames}",
    ]
    if result.targets:
        lines.append("targets (estimate = back-projected from the detections):")
        matches = list(truth_matches) if truth_matches is not None else [None] * len(result.targets)
        for target, match in zip(result.targets, matches):
            x, y, z = target.position
            lines.append(
                f"  - {target.class_id:<12} estimate ({x:8.2f}, {y:8.2f}, {z:6.2f})  "
                f"hits={target.hits}  best_score={target.best_score:.2f}")
            if match is not None:
                lines.append(
                    f"    {'':<12} truth    ({match.position[0]:8.2f}, {match.position[1]:8.2f}, "
                    f"{match.position[2]:6.2f})  {match.target_id} [{match.class_id}]  "
                    f"error {match.error_m:.2f} m")
            elif truth_matches is not None:
                lines.append(f"    {'':<12} truth    no target nearby (possible false positive)")
    else:
        lines.append("targets: none confirmed")
    lines.append("")
    lines.append(format_path(result.path))
    return "\n".join(lines)


def format_progress(progress: SearchProgress) -> str:
    """Render a one-line in-flight mission status."""
    remaining = ("queue not reported yet" if progress.waypoints_remaining is None
                 else f"{progress.waypoints_completed}/{progress.waypoints_total} waypoint(s)")
    found = (", ".join(f"{target.class_id} at ({target.position[0]:.1f}, {target.position[1]:.1f})"
                       for target in progress.targets) or "none yet")
    return (f"Mission running {progress.elapsed_sec:.0f}s: {remaining}, "
            f"{progress.detector_frames} detector frame(s), found: {found}")


def format_path(path: Path) -> str:
    """Render a sparse ROS path as readable ENU waypoints for the result panel."""
    header = f"{len(path.poses)} waypoint(s), frame={path.header.frame_id or '<unset>'}"
    lines = [header]
    for index, pose_stamped in enumerate(path.poses, start=1):
        pose = pose_stamped.pose
        heading = _yaw_deg(pose_stamped)
        lines.append(
            f"{index:03d}: x={pose.position.x:.2f}, y={pose.position.y:.2f}, "
            f"z={pose.position.z:.2f}, yaw={heading:.1f} deg")
    return "\n".join(lines)


def _yaw_deg(pose: PoseStamped) -> float:
    quaternion = pose.pose.orientation
    yaw = atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
    return degrees(yaw) % 360.0
