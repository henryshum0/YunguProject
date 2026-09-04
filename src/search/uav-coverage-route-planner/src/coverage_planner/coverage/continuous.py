"""Continuous image sampling and flight-command construction."""

from __future__ import annotations

from dataclasses import replace
from itertools import pairwise
from math import acos, atan2, ceil, degrees, hypot

from coverage_planner.camera import (
    ground_footprint_dimensions,
    ground_footprint_polygon,
)
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.flight import (
    ContinuousFlightPlan,
    CoverageLane,
    FlightWaypoint,
    RouteSegment,
    SegmentKind,
)
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.semantic_map import SemanticMap
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.visibility import visible_detection_ground

# Repeatedly subtracting almost coincident camera footprints can leave edges at
# floating-point precision.  A sub-micrometre precision grid keeps GEOS overlay
# operations deterministic without changing planning geometry at metre scale.
_OVERLAY_GRID_SIZE_M = 1e-6


def build_continuous_flight_plan(
    route: tuple[Waypoint, ...], *, camera: CameraConfig, flight_altitude_m: float,
    ground_elevation_m: float,
    coverage_speed_mps: float, connector_speed_mps: float,
    obstacle_speed_mps: float, return_speed_mps: float,
    control_point_spacing_m: float = 10.0,
    capture_region: Polygonal | None = None,
    semantic_map: SemanticMap | None = None,
) -> tuple[ContinuousFlightPlan, dict[str, Polygonal]]:
    """Convert a route into commands and sample its continuous visibility sweep."""
    if control_point_spacing_m <= 0:
        raise ValueError("control_point_spacing_m must be greater than zero")
    route = _compact_route(route)
    dimensions = ground_footprint_dimensions(
        camera, flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m)
    # Visibility is a geometric sweep of the flown route.  Sampling is only a
    # numerical approximation for building occlusion; it is independent of
    # video frame rate and commanded speed.
    geometry_sweep_step_m = max(0.5, min(2.0, dimensions.length_m / 8.0))
    commanded_waypoints: list[FlightWaypoint] = []
    segments: list[RouteSegment] = []
    footprints: dict[str, Polygonal] = {}

    for sequence, waypoint in enumerate(route, 1):
        if sequence < len(route):
            heading = _heading(waypoint, route[sequence])
            kind = _segment_kind(waypoint, route[sequence])
            speed = _speed(kind, coverage_speed_mps, connector_speed_mps,
                           obstacle_speed_mps, return_speed_mps)
        else:
            heading = commanded_waypoints[-1].heading_deg if commanded_waypoints else 0.0
            speed = 0.0
        commanded_waypoints.append(FlightWaypoint(
            id=f"fp_{sequence:04d}", sequence=sequence, x=waypoint.x, y=waypoint.y,
            z=waypoint.z, heading_deg=heading, speed_mps=speed))

    for sequence, (start, end) in enumerate(pairwise(route), 1):
        kind = _segment_kind(start, end)
        speed = _speed(kind, coverage_speed_mps, connector_speed_mps,
                       obstacle_speed_mps, return_speed_mps)
        heading = _heading(start, end)
        length = hypot(end.x - start.x, end.y - start.y)
        segment_id = f"segment_{sequence:04d}"
        segments.append(RouteSegment(
            id=segment_id, sequence=sequence, kind=kind,
            start_waypoint_id=commanded_waypoints[sequence - 1].id,
            end_waypoint_id=commanded_waypoints[sequence].id,
            heading_deg=heading, speed_mps=speed, length_m=length,
            detection_enabled=True,
            source_scan_line_index=end.scan_line_index if kind == "coverage_lane" else None,
            source_scan_segment_index=end.scan_segment_index if kind == "coverage_lane" else None,
            source_coverage_cell_index=(
                end.coverage_cell_index if kind == "coverage_lane" else None)))
        interval_count = max(
            1, ceil(length / geometry_sweep_step_m - 1e-12)) if length else 1
        for sample_index in range(interval_count + 1):
            fraction = sample_index / interval_count
            point = (start.x + fraction * (end.x - start.x),
                     start.y + fraction * (end.y - start.y))
            sample_id = f"{segment_id}_image_{sample_index:04d}"
            footprints[sample_id] = _detection_ground(
                camera, point=point, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=heading,
                semantic_map=semantic_map)

    covered_route_indices = {
        index
        for index, segment in enumerate(segments)
        if segment.detection_enabled
        for index in (index, index + 1)
    }
    for index, waypoint in enumerate(route):
        if (not waypoint.capture
                or (index in covered_route_indices and not waypoint.is_completion)):
            continue
        sample_id = f"segment_point_{index + 1:04d}_image_0000"
        footprints[sample_id] = _detection_ground(
            camera, point=(waypoint.x, waypoint.y),
            flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, yaw_deg=waypoint.yaw_deg,
            semantic_map=semantic_map)

    commanded_waypoints, segments = _densify_commands(
        commanded_waypoints, segments, control_point_spacing_m)
    commanded_waypoints = _annotate_sharp_turns(commanded_waypoints)
    lanes = _group_lanes(segments)
    return ContinuousFlightPlan(
        control_point_spacing_m=control_point_spacing_m, lane_overlap=camera.side_overlap,
        target_width_m=camera.target_width_m, target_length_m=camera.target_length_m,
        target_height_m=camera.target_height_m,
        image_boundary_margin_ratio=camera.image_boundary_margin_ratio,
        waypoints=tuple(commanded_waypoints),
        route_segments=tuple(segments), lanes=lanes,
        visibility_sample_count=len(footprints)), footprints


def _detection_ground(
    camera: CameraConfig, *, point: tuple[float, float], flight_altitude_m: float,
    ground_elevation_m: float, yaw_deg: float, semantic_map: SemanticMap | None,
) -> Polygonal:
    if semantic_map is not None:
        return visible_detection_ground(
            camera, center_enu_m=point, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg,
            semantic_map=semantic_map)
    return ground_footprint_polygon(
        camera, center_enu_m=point, flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg)


def _densify_commands(
    waypoints: list[FlightWaypoint], segments: list[RouteSegment], maximum_spacing_m: float,
) -> tuple[list[FlightWaypoint], list[RouteSegment]]:
    """Insert equally spaced control points along every straight route segment."""
    if not segments:
        return waypoints, segments
    by_id = {waypoint.id: waypoint for waypoint in waypoints}
    dense_waypoints: list[FlightWaypoint] = []
    dense_segments: list[RouteSegment] = []
    for source in segments:
        start = by_id[source.start_waypoint_id]
        end = by_id[source.end_waypoint_id]
        interval_count = max(1, ceil(source.length_m / maximum_spacing_m - 1e-12))
        if dense_waypoints:
            dense_waypoints[-1] = replace(
                dense_waypoints[-1], heading_deg=source.heading_deg, speed_mps=source.speed_mps)
        else:
            dense_waypoints.append(replace(
                start, heading_deg=source.heading_deg, speed_mps=source.speed_mps))
        for interval in range(1, interval_count + 1):
            fraction = interval / interval_count
            dense_waypoints.append(FlightWaypoint(
                id="", sequence=0,
                x=start.x + fraction * (end.x - start.x),
                y=start.y + fraction * (end.y - start.y),
                z=start.z + fraction * (end.z - start.z),
                heading_deg=source.heading_deg, speed_mps=source.speed_mps))
            dense_segments.append(replace(
                source, id="", sequence=0, start_waypoint_id="", end_waypoint_id="",
                length_m=source.length_m / interval_count))
    dense_waypoints[-1] = replace(dense_waypoints[-1], speed_mps=0.0)
    dense_waypoints = [replace(
        waypoint, id=f"fp_{index:04d}", sequence=index)
        for index, waypoint in enumerate(dense_waypoints, 1)]
    dense_segments = [replace(
        segment, id=f"segment_{index:04d}", sequence=index,
        start_waypoint_id=dense_waypoints[index - 1].id,
        end_waypoint_id=dense_waypoints[index].id)
        for index, segment in enumerate(dense_segments, 1)]
    return dense_waypoints, dense_segments


def _annotate_sharp_turns(
    waypoints: list[FlightWaypoint], *, minimum_internal_angle_deg: float = 60.0,
) -> list[FlightWaypoint]:
    """Require a controlled stop/yaw at corners unsafe for continuous tracking."""
    for index in range(1, len(waypoints) - 1):
        previous, current, following = waypoints[index - 1:index + 2]
        incoming = (previous.x - current.x, previous.y - current.y)
        outgoing = (following.x - current.x, following.y - current.y)
        incoming_length = hypot(*incoming)
        outgoing_length = hypot(*outgoing)
        if incoming_length <= 1e-9 or outgoing_length <= 1e-9:
            continue
        cosine = max(-1.0, min(1.0, (
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
        ) / (incoming_length * outgoing_length)))
        internal_angle = degrees(acos(cosine))
        if internal_angle < minimum_internal_angle_deg:
            waypoints[index] = replace(current, turn_in_place=True, hold_time_s=0.5)
    return waypoints


def _segment_kind(start: Waypoint, end: Waypoint) -> SegmentKind:
    if end.id == "wp_home_return":
        return "return_home"
    same_lane = (
        start.capture and end.capture
        and start.scan_line_index is not None
        and start.scan_segment_index is not None
        and start.scan_line_index == end.scan_line_index
        and start.scan_segment_index == end.scan_segment_index
    )
    if same_lane:
        return "coverage_lane"
    if (not start.capture or not end.capture) and start.id != "wp_start":
        return "obstacle_avoidance"
    return "connector"


def _speed(kind: SegmentKind, coverage: float, connector: float,
           obstacle: float, return_speed: float) -> float:
    speed = {"coverage_lane": coverage, "connector": connector,
             "obstacle_avoidance": obstacle, "return_home": return_speed}[kind]
    if speed <= 0:
        raise ValueError(f"{kind} speed must be greater than zero")
    return speed


def _heading(start: Waypoint, end: Waypoint) -> float:
    if start.x == end.x and start.y == end.y:
        return 0.0
    return degrees(atan2(end.x - start.x, end.y - start.y)) % 360.0


def _group_lanes(segments: list[RouteSegment]) -> tuple[CoverageLane, ...]:
    groups: list[list[RouteSegment]] = []
    for segment in segments:
        if segment.kind != "coverage_lane":
            continue
        if (not groups or
                groups[-1][-1].source_scan_line_index != segment.source_scan_line_index or
                groups[-1][-1].source_scan_segment_index != segment.source_scan_segment_index):
            groups.append([])
        groups[-1].append(segment)
    return tuple(CoverageLane(
        id=f"lane_{index:03d}", sequence=index,
        heading_deg=group[0].heading_deg, speed_mps=group[0].speed_mps,
        route_segment_ids=tuple(item.id for item in group),
        length_m=sum(item.length_m for item in group),
    ) for index, group in enumerate(groups, 1))


def _compact_route(route: tuple[Waypoint, ...]) -> tuple[Waypoint, ...]:
    """Reduce dense coverage samples while preserving turns and route semantics."""
    deduplicated = [route[0]] if route else []
    for waypoint in route[1:]:
        previous = deduplicated[-1]
        if hypot(waypoint.x - previous.x, waypoint.y - previous.y) <= 1e-8:
            deduplicated[-1] = waypoint
        else:
            deduplicated.append(waypoint)
    route = tuple(deduplicated)
    if len(route) < 3:
        return route
    compacted = [route[0]]
    for index in range(1, len(route) - 1):
        previous = compacted[-1]
        current = route[index]
        following = route[index + 1]
        same_lane = (
            previous.capture and current.capture and following.capture
            and previous.scan_line_index == current.scan_line_index == following.scan_line_index
            and previous.scan_segment_index == current.scan_segment_index
            == following.scan_segment_index
        )
        if same_lane and _is_collinear(previous, current, following):
            continue
        compacted.append(current)
    compacted.append(route[-1])
    return tuple(compacted)


def _is_collinear(a: Waypoint, b: Waypoint, c: Waypoint) -> bool:
    cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)
    return abs(cross) <= 1e-8
