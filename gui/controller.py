"""ROS-facing controller used by the standalone skills test GUI."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from skills import NavigateSkill, PlanSearchPrimitive, SearchSkill, SkillRuntimeConfig


@dataclass(frozen=True, slots=True)
class ConnectionSettings:
    config: SkillRuntimeConfig
    timeout_sec: float


class SkillController:
    """Create configured skills and publish the existing flight-control commands."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._flight_publishers = {}

    def takeoff(self, settings: ConnectionSettings) -> None:
        self._publish_flight_command(settings.config.offboard.takeoff_topic)

    def land(self, settings: ConnectionSettings) -> None:
        self._publish_flight_command(settings.config.offboard.land_topic)

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

    def _publish_flight_command(self, topic: str) -> None:
        publisher = self._flight_publishers.get(topic)
        if publisher is None:
            publisher = self._node.create_publisher(
                Bool, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            self._flight_publishers[topic] = publisher
        command = Bool()
        command.data = True
        publisher.publish(command)


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
