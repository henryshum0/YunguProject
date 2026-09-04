"""Strict JSON parsers for planner startup and reusable map files."""

from __future__ import annotations

import json
from math import isfinite
from pathlib import Path
import re
from typing import Any

from shapely.geometry import Polygon
from shapely.validation import explain_validity

from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.config import (
    FlightConfig,
    MapConfig,
    OccupiedArea,
    OutputTopics,
    Origin,
    PlannerConfig,
    StartupConfig,
)


class ConfigError(ValueError):
    """Raised when a planner or map JSON file violates its public schema."""


_MISSING = object()
_ABSOLUTE_TOPIC_NAME = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$")


def load_config(path: str | Path) -> StartupConfig:
    """Load a planner JSON and the map JSON it references."""
    source = Path(path).expanduser()
    planner_payload = _load_json(source, "config_file")
    root = _object(planner_payload, "root")
    map_reference = _string(root, "map_file", "root")
    map_source = Path(map_reference).expanduser()
    if not map_source.is_absolute():
        map_source = source.parent / map_source
    try:
        map_config = parse_map_config(_load_json(map_source, "map_file"))
    except ConfigError as exc:
        raise ConfigError(f"invalid map_file '{map_source}': {exc}") from exc
    try:
        return parse_config(planner_payload, map_config, map_file=str(map_source))
    except ConfigError as exc:
        raise ConfigError(f"invalid config_file '{source}': {exc}") from exc


def parse_map_config(payload: Any) -> MapConfig:
    """Validate a reusable map JSON (schema version 1.0)."""
    root = _object(payload, "map root")
    _fields(
        root,
        "map root",
        allowed={"schema_version", "origin", "occupied_areas"},
        required={"schema_version", "origin"},
    )
    schema_version = _string(root, "schema_version", "map root")
    if schema_version != "1.0":
        raise ConfigError("map schema_version must be '1.0'")
    origin_data = _nested_object(root, "origin", "map root")
    _fields(origin_data, "map origin", allowed={"x", "y"}, required={"x", "y"})
    return MapConfig(
        schema_version=schema_version,
        origin=Origin(
            _number(origin_data, "x", "map origin"),
            _number(origin_data, "y", "map origin"),
        ),
        occupied_areas=_occupied_areas(root.get("occupied_areas", []), "occupied_areas"),
    )


def parse_config(
    payload: Any,
    map_config: MapConfig | Any,
    *,
    map_file: str | None = None,
) -> StartupConfig:
    """Validate a planner JSON and combine it with an already loaded map."""
    root = _object(payload, "root")
    _fields(
        root,
        "root",
        allowed={
            "schema_version", "map_file", "frame_id", "flight",
            "camera", "planner", "output_topics",
        },
        required={"schema_version", "map_file", "flight", "camera"},
    )
    schema_version = _string(root, "schema_version", "root")
    if schema_version != "1.2":
        raise ConfigError("schema_version must be '1.2'")
    map_reference = _string(root, "map_file", "root")
    parsed_map = map_config if isinstance(map_config, MapConfig) else parse_map_config(map_config)
    frame_id = _string(root, "frame_id", "root", default="map")

    flight_data = _nested_object(root, "flight", "root")
    _fields(
        flight_data,
        "flight",
        allowed={"altitude_m", "ground_elevation_m", "horizontal_clearance_m"},
        required={"altitude_m"},
    )
    flight = FlightConfig(
        altitude_m=_number(flight_data, "altitude_m", "flight"),
        ground_elevation_m=_number(
            flight_data, "ground_elevation_m", "flight", default=0.0),
        horizontal_clearance_m=_number(
            flight_data, "horizontal_clearance_m", "flight", default=3.0),
    )
    if flight.horizontal_clearance_m < 0.0:
        raise ConfigError("flight.horizontal_clearance_m cannot be negative")

    camera_data = _nested_object(root, "camera", "root")
    _fields(
        camera_data,
        "camera",
        allowed={
            "horizontal_fov_deg", "vertical_fov_deg", "side_overlap",
            "target_width_m", "target_length_m", "target_height_m",
            "image_boundary_margin_ratio",
        },
        required={"horizontal_fov_deg", "vertical_fov_deg", "side_overlap"},
    )
    try:
        camera = CameraConfig(
            horizontal_fov_deg=_number(camera_data, "horizontal_fov_deg", "camera"),
            vertical_fov_deg=_number(camera_data, "vertical_fov_deg", "camera"),
            side_overlap=_number(camera_data, "side_overlap", "camera"),
            target_width_m=_number(camera_data, "target_width_m", "camera", default=0.0),
            target_length_m=_number(camera_data, "target_length_m", "camera", default=0.0),
            target_height_m=_number(camera_data, "target_height_m", "camera", default=0.0),
            image_boundary_margin_ratio=_number(
                camera_data, "image_boundary_margin_ratio", "camera", default=0.0),
        )
    except ValueError as exc:
        raise ConfigError(f"camera: {exc}") from exc

    planner = _planner_config(_object(root.get("planner", {}), "planner"))
    topic_data = _object(root.get("output_topics", {}), "output_topics")
    _fields(topic_data, "output_topics", allowed={"waypoints", "markers"}, required=set())
    output_topics = OutputTopics(
        waypoints=_absolute_topic_name(topic_data, "waypoints", "/coverage_planner/waypoints"),
        markers=_absolute_topic_name(topic_data, "markers", "/coverage_planner/markers"),
    )
    if output_topics.waypoints == output_topics.markers:
        raise ConfigError("output_topics.waypoints and output_topics.markers must differ")
    if flight.altitude_m <= flight.ground_elevation_m + camera.target_height_m:
        raise ConfigError("flight.altitude_m must exceed ground elevation plus target height")
    return StartupConfig(
        schema_version=schema_version,
        map_file=map_file or map_reference,
        frame_id=frame_id,
        origin=parsed_map.origin,
        occupied_areas=parsed_map.occupied_areas,
        flight=flight,
        camera=camera,
        planner=planner,
        output_topics=output_topics,
    )


def _load_json(source: Path, label: str) -> Any:
    try:
        return json.loads(
            source.read_text(encoding="utf-8"),
            parse_constant=lambda value: _reject_json_constant(value),
        )
    except OSError as exc:
        raise ConfigError(f"cannot read {label} '{source}': {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(
            f"invalid JSON in {label} '{source}' at line {exc.lineno}, "
            f"column {exc.colno}: {exc.msg}") from exc


def _occupied_areas(value: Any, path: str) -> tuple[OccupiedArea, ...]:
    if not isinstance(value, list):
        raise ConfigError(f"{path} must be an array")
    occupied: list[OccupiedArea] = []
    for index, item_value in enumerate(value):
        item_path = f"{path}[{index}]"
        item = _object(item_value, item_path)
        _fields(item, item_path, allowed={"id", "points"}, required={"id", "points"})
        occupied.append(OccupiedArea(
            _string(item, "id", item_path),
            _polygon_points(item["points"], f"{item_path}.points"),
        ))
    ids = tuple(area.id for area in occupied)
    if len(ids) != len(set(ids)):
        duplicates = sorted({identifier for identifier in ids if ids.count(identifier) > 1})
        raise ConfigError(f"occupied_areas IDs must be unique; duplicates: {duplicates}")
    return tuple(occupied)


def _planner_config(data: dict[str, Any]) -> PlannerConfig:
    _fields(
        data,
        "planner",
        allowed={
            "coverage_generation_method", "scan_direction_deg",
            "route_optimization_method", "completion_strategy",
            "minimum_coverage_ratio", "return_to_origin",
        },
        required=set(),
    )
    coverage_method = _string(
        data, "coverage_generation_method", "planner", default="global_scanline")
    if coverage_method not in {"global_scanline", "bcd"}:
        raise ConfigError(
            "planner.coverage_generation_method must be 'global_scanline' or 'bcd'")
    scan_direction = _nullable_number(data, "scan_direction_deg", "planner", default=None)
    if scan_direction is not None and not 0.0 <= scan_direction < 360.0:
        raise ConfigError("planner.scan_direction_deg must be in [0, 360)")
    route_method = _string(data, "route_optimization_method", "planner", default="auto")
    if route_method not in {"greedy", "two_opt", "or_opt", "heuristic", "exact", "auto"}:
        raise ConfigError("planner.route_optimization_method is unsupported")
    completion = _string(data, "completion_strategy", "planner", default="local_insertion")
    if completion not in {"full_greedy", "local_insertion"}:
        raise ConfigError("planner.completion_strategy is unsupported")
    minimum_coverage = _number(data, "minimum_coverage_ratio", "planner", default=0.99)
    if not 0.0 < minimum_coverage <= 1.0:
        raise ConfigError("planner.minimum_coverage_ratio must be in (0, 1]")
    return_to_origin = data.get("return_to_origin", True)
    if type(return_to_origin) is not bool:
        raise ConfigError("planner.return_to_origin must be a boolean")
    return PlannerConfig(
        coverage_generation_method=coverage_method,  # type: ignore[arg-type]
        scan_direction_deg=scan_direction,
        route_optimization_method=route_method,  # type: ignore[arg-type]
        completion_strategy=completion,  # type: ignore[arg-type]
        minimum_coverage_ratio=minimum_coverage,
        return_to_origin=return_to_origin,
    )


def _reject_json_constant(value: str) -> None:
    raise ConfigError(f"non-finite JSON number {value!r} is not allowed")


def _object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ConfigError(f"{path} must be an object")
    return value


def _nested_object(data: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    return _object(data[key], f"{path}.{key}" if path != "root" else key)


def _fields(
    data: dict[str, Any], path: str, *, allowed: set[str], required: set[str],
) -> None:
    unknown = sorted(set(data).difference(allowed))
    if unknown:
        raise ConfigError(f"{path} contains unknown fields: {unknown}")
    missing = sorted(required.difference(data))
    if missing:
        raise ConfigError(f"{path} is missing required fields: {missing}")


def _string(
    data: dict[str, Any], key: str, path: str, *, default: object = _MISSING,
) -> str:
    value = data.get(key, default)
    if value is _MISSING:
        raise ConfigError(f"{path}.{key} is required")
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{path}.{key} must be a non-empty string")
    return value


def _number(
    data: dict[str, Any], key: str, path: str, *, default: object = _MISSING,
) -> float:
    value = data.get(key, default)
    if value is _MISSING:
        raise ConfigError(f"{path}.{key} is required")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ConfigError(f"{path}.{key} must be a finite number")
    return float(value)


def _nullable_number(
    data: dict[str, Any], key: str, path: str, *, default: float | None,
) -> float | None:
    value = data.get(key, default)
    if value is None:
        return None
    return _number({key: value}, key, path)


def _absolute_topic_name(data: dict[str, Any], key: str, default: str) -> str:
    topic = _string(data, key, "output_topics", default=default)
    if not _ABSOLUTE_TOPIC_NAME.fullmatch(topic):
        raise ConfigError(f"output_topics.{key} must be an absolute ROS topic name")
    return topic


def _polygon_points(value: Any, path: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list):
        raise ConfigError(f"{path} must be an array")
    points: list[tuple[float, float]] = []
    for index, point in enumerate(value):
        if not isinstance(point, list) or len(point) != 2:
            raise ConfigError(f"{path}[{index}] must be a two-number array")
        points.append((
            _number({"x": point[0]}, "x", f"{path}[{index}]"),
            _number({"y": point[1]}, "y", f"{path}[{index}]"),
        ))
    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3 or len(set(points)) < 3:
        raise ConfigError(f"{path} must contain at least three distinct points")
    polygon = Polygon(points)
    if polygon.is_empty or polygon.area <= 0.0 or not polygon.is_valid:
        raise ConfigError(f"{path} must form a valid simple polygon: {explain_validity(polygon)}")
    return tuple(points)
