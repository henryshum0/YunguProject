"""Dependency-free map geometry helpers for the skills test GUI."""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path
from typing import Any, Iterable


Point = tuple[float, float]


class MapLoadError(ValueError):
    """Raised when a planner-compatible map JSON cannot be displayed."""


@dataclass(frozen=True, slots=True)
class OccupiedArea:
    identifier: str
    points: tuple[Point, ...]


@dataclass(frozen=True, slots=True)
class PlannerMap:
    source: Path
    origin: Point
    occupied_areas: tuple[OccupiedArea, ...]


@dataclass(frozen=True, slots=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True, slots=True)
class Viewport:
    """Uniform ENU-to-canvas transform with an upward-pointing ENU y axis."""

    bounds: Bounds
    width: float
    height: float
    padding: float
    scale: float
    offset_x: float
    offset_y: float

    def to_canvas(self, point: Point) -> Point:
        x, y = point
        return self.offset_x + (x - self.bounds.min_x) * self.scale, (
            self.offset_y + (self.bounds.max_y - y) * self.scale)

    def to_enu(self, point: Point) -> Point:
        x, y = point
        return (
            self.bounds.min_x + (x - self.offset_x) / self.scale,
            self.bounds.max_y - (y - self.offset_y) / self.scale,
        )


def load_planner_map(path: str | Path) -> PlannerMap:
    """Load the strict map schema (1.0) used by coverage_planner."""
    source = Path(path).expanduser()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise MapLoadError(f"cannot read map JSON '{source}': {exc}") from exc
    except json.JSONDecodeError as exc:
        raise MapLoadError(
            f"invalid JSON in '{source}' at line {exc.lineno}, column {exc.colno}: {exc.msg}") from exc

    root = _object(payload, "map root")
    _fields(root, "map root", {"schema_version", "origin", "occupied_areas"},
            {"schema_version", "origin"})
    if root["schema_version"] != "1.0":
        raise MapLoadError("map schema_version must be '1.0'")
    origin_data = _object(root["origin"], "origin")
    _fields(origin_data, "origin", {"x", "y"}, {"x", "y"})
    origin = (_number(origin_data["x"], "origin.x"), _number(origin_data["y"], "origin.y"))

    raw_areas = root.get("occupied_areas", [])
    if not isinstance(raw_areas, list):
        raise MapLoadError("occupied_areas must be an array")
    occupied: list[OccupiedArea] = []
    identifiers: set[str] = set()
    for index, raw_area in enumerate(raw_areas):
        label = f"occupied_areas[{index}]"
        area = _object(raw_area, label)
        _fields(area, label, {"id", "points"}, {"id", "points"})
        identifier = area["id"]
        if not isinstance(identifier, str) or not identifier:
            raise MapLoadError(f"{label}.id must be a non-empty string")
        if identifier in identifiers:
            raise MapLoadError(f"occupied_areas IDs must be unique: {identifier!r}")
        identifiers.add(identifier)
        occupied.append(OccupiedArea(identifier, _polygon_points(area["points"], f"{label}.points")))
    return PlannerMap(source=source, origin=origin, occupied_areas=tuple(occupied))


def rectangle_from_clicks(first: Point, second: Point) -> tuple[Point, Point, Point, Point]:
    """Return service-ready SW, SE, NE, NW ENU corners from opposite clicks."""
    first = _finite_point(first, "first map click")
    second = _finite_point(second, "second map click")
    min_x, max_x = sorted((first[0], second[0]))
    min_y, max_y = sorted((first[1], second[1]))
    if min_x == max_x or min_y == max_y:
        raise ValueError("map clicks must define a non-zero-area rectangle")
    return ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y))


def route_points(path: Any) -> tuple[Point, ...]:
    """Extract finite ENU points from a ROS ``nav_msgs/Path``-like object."""
    try:
        poses = path.poses
    except AttributeError as exc:
        raise ValueError("planner response does not contain a path") from exc
    return tuple(_finite_point(
        (pose.pose.position.x, pose.pose.position.y), f"route waypoint {index}")
        for index, pose in enumerate(poses, start=1))


def bounds_for(map_data: PlannerMap, *overlays: Iterable[Point]) -> Bounds:
    """Return bounds covering map geometry, origin, and optional overlays."""
    points: list[Point] = [map_data.origin]
    for area in map_data.occupied_areas:
        points.extend(area.points)
    for overlay in overlays:
        points.extend(_finite_point(point, "overlay point") for point in overlay)
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    return Bounds(min_x, max_x, min_y, max_y)


def make_viewport(bounds: Bounds, width: float, height: float, padding: float = 24.0) -> Viewport:
    """Create a centred, uniform viewport that preserves ENU orientation."""
    if width <= 2.0 * padding or height <= 2.0 * padding:
        raise ValueError("map canvas is too small to render")
    span_x = max(bounds.max_x - bounds.min_x, 1.0)
    span_y = max(bounds.max_y - bounds.min_y, 1.0)
    scale = min((width - 2.0 * padding) / span_x, (height - 2.0 * padding) / span_y)
    rendered_width, rendered_height = span_x * scale, span_y * scale
    return Viewport(
        bounds=bounds,
        width=width,
        height=height,
        padding=padding,
        scale=scale,
        offset_x=(width - rendered_width) / 2.0,
        offset_y=(height - rendered_height) / 2.0,
    )


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise MapLoadError(f"{label} must be an object")
    return value


def _fields(data: dict[str, Any], label: str, allowed: set[str], required: set[str]) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise MapLoadError(f"{label} has unknown fields: {unknown}")
    missing = sorted(required - set(data))
    if missing:
        raise MapLoadError(f"{label} is missing required fields: {missing}")


def _number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not isfinite(float(value)):
        raise MapLoadError(f"{label} must be a finite number")
    return float(value)


def _polygon_points(value: Any, label: str) -> tuple[Point, ...]:
    if not isinstance(value, list):
        raise MapLoadError(f"{label} must be an array")
    points: list[Point] = []
    for index, coordinate in enumerate(value):
        if not isinstance(coordinate, list) or len(coordinate) != 2:
            raise MapLoadError(f"{label}[{index}] must be [x, y]")
        points.append((_number(coordinate[0], f"{label}[{index}][0]"),
                       _number(coordinate[1], f"{label}[{index}][1]")))
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3 or len(set(points)) < 3:
        raise MapLoadError(f"{label} must contain at least three distinct points")
    if _signed_area(points) == 0.0 or _self_intersects(points):
        raise MapLoadError(f"{label} must form a valid simple polygon")
    return tuple(points)


def _finite_point(point: Point, label: str) -> Point:
    x, y = point
    if not isfinite(float(x)) or not isfinite(float(y)):
        raise ValueError(f"{label} must contain finite ENU coordinates")
    return float(x), float(y)


def _signed_area(points: list[Point]) -> float:
    return sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])) / 2.0


def _self_intersects(points: list[Point]) -> bool:
    edges = list(zip(points, points[1:] + points[:1]))
    for left, first in enumerate(edges):
        for right, second in enumerate(edges[left + 1:], start=left + 1):
            if right == left + 1 or (left == 0 and right == len(edges) - 1):
                continue
            if _segments_intersect(*first, *second):
                return True
    return False


def _segments_intersect(first_a: Point, first_b: Point, second_a: Point, second_b: Point) -> bool:
    def orientation(a: Point, b: Point, c: Point) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    one = orientation(first_a, first_b, second_a)
    two = orientation(first_a, first_b, second_b)
    three = orientation(second_a, second_b, first_a)
    four = orientation(second_a, second_b, first_b)
    return (one > 0.0) != (two > 0.0) and (three > 0.0) != (four > 0.0)
