"""Reusable ENU/NED waypoint and ROS pose conversion helpers."""

from __future__ import annotations

from collections.abc import Sequence
from math import cos, isfinite, radians, sin
from typing import Literal, TypeAlias

from geometry_msgs.msg import PoseStamped


CoordinateFrame: TypeAlias = Literal["enu", "ned"]
Waypoint: TypeAlias = tuple[float, float, float, float]
WaypointInput: TypeAlias = Waypoint | Sequence[Waypoint]


def normalize_waypoints(waypoints: object) -> tuple[Waypoint, ...]:
    """Validate a single waypoint or waypoint sequence into a non-empty tuple."""
    if _is_coordinate_sequence(waypoints):
        return (_validate_waypoint(waypoints),)
    if not _is_sequence(waypoints) or not waypoints:
        raise ValueError("waypoints must be one waypoint or a non-empty sequence of waypoints")
    if all(not _is_sequence(item) for item in waypoints):
        raise ValueError("one waypoint must be an (x, y, z, heading_deg) sequence")
    return tuple(_validate_waypoint(item) for item in waypoints)


def to_enu_waypoints(waypoints: object, *, frame: str) -> tuple[Waypoint, ...]:
    """Validate and convert frame-native coordinate waypoints to ENU."""
    normalized_frame = normalize_frame(frame)
    converted = normalize_waypoints(waypoints)
    if normalized_frame == "enu":
        return converted
    return tuple(ned_waypoint_to_enu(waypoint) for waypoint in converted)


def normalize_frame(frame: str) -> CoordinateFrame:
    """Return a supported coordinate frame name in lowercase."""
    if not isinstance(frame, str) or frame.lower() not in {"enu", "ned"}:
        raise ValueError("frame must be either 'enu' or 'ned'")
    return frame.lower()  # type: ignore[return-value]


def ned_waypoint_to_enu(waypoint: Waypoint) -> Waypoint:
    """Convert ``(north, east, down, yaw)`` to ENU frame-native coordinates."""
    north, east, down, yaw_deg = _validate_waypoint(waypoint)
    return east, north, -down, (90.0 - yaw_deg) % 360.0


def enu_yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """Build a ROS quaternion for an ENU yaw in degrees."""
    normalized_yaw = _finite_float(yaw_deg, "heading_deg") % 360.0
    yaw_rad = radians(normalized_yaw)
    return 0.0, 0.0, sin(yaw_rad / 2.0), cos(yaw_rad / 2.0)


def pose_stamped_from_enu_waypoint(waypoint: Waypoint, *, frame_id: str) -> PoseStamped:
    """Build a ROS ENU pose from ``(east, north, up, yaw_deg)``."""
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("frame_id must be a non-empty string")
    east, north, up, yaw_deg = _validate_waypoint(waypoint)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = east
    pose.pose.position.y = north
    pose.pose.position.z = up
    x, y, z, w = enu_yaw_quaternion(yaw_deg)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def _validate_waypoint(value: object) -> Waypoint:
    if not _is_coordinate_sequence(value):
        raise ValueError("each waypoint must be an (x, y, z, heading_deg) sequence")
    x, y, z, heading_deg = value
    return (
        _finite_float(x, "x"),
        _finite_float(y, "y"),
        _finite_float(z, "z"),
        _finite_float(heading_deg, "heading_deg") % 360.0,
    )


def _is_coordinate_sequence(value: object) -> bool:
    return _is_sequence(value) and len(value) == 4 and all(
        not _is_sequence(item) for item in value)


def _is_sequence(value: object) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray))


def _finite_float(value: object, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"waypoint {field_name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"waypoint {field_name} must be a finite number") from exc
    if not isfinite(number):
        raise ValueError(f"waypoint {field_name} must be a finite number")
    return number
