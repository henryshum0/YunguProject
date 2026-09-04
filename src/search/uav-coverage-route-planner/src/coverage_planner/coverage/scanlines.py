"""Boustrophedon capture waypoint generation from polygon intersections."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import ceil

from shapely import affinity
from shapely.geometry import GeometryCollection, LineString, MultiLineString

from coverage_planner.camera import (
    ground_footprint_dimensions,
    ground_footprint_polygon,
)
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import ScanSegment, Waypoint


class ScanlinePlanningError(ValueError):
    """Raised when scan lines cannot be generated from the requested settings."""


@dataclass(frozen=True, slots=True)
class CoveragePlan:
    """Coverage lanes and their uniformly sampled reference waypoints."""

    scan_direction_deg: float
    scan_segments: tuple[ScanSegment, ...]
    capture_waypoints: tuple[Waypoint, ...]

    @property
    def coverage_lanes(self) -> tuple[ScanSegment, ...]:
        return self.scan_segments

    @property
    def reference_waypoints(self) -> tuple[Waypoint, ...]:
        return self.capture_waypoints


CapturePlan = CoveragePlan


def generate_capture_plan(
    effective_geometry: Polygonal,
    *,
    camera: CameraConfig,
    flight_altitude_m: float,
    ground_elevation_m: float,
    scan_direction_deg: float,
) -> CoveragePlan:
    """Intersect parallel lines with free ground and sample each resulting segment."""
    if effective_geometry.is_empty:
        return CoveragePlan(scan_direction_deg % 360.0, (), ())
    if not effective_geometry.is_valid:
        raise ScanlinePlanningError("effective search geometry must be valid")
    dimensions = ground_footprint_dimensions(
        camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
    )
    direction_deg = scan_direction_deg % 360.0
    rotation_deg = direction_deg - 90.0
    scan_geometry = affinity.rotate(effective_geometry, rotation_deg, origin=(0.0, 0.0))
    _, min_y, _, max_y = scan_geometry.bounds
    line_positions = _scan_line_positions(
        min_y,
        max_y,
        footprint_width_m=dimensions.width_m,
        spacing_m=dimensions.scan_line_spacing_m,
    )
    return generate_capture_plan_on_scan_lines(
        effective_geometry,
        camera=camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
        scan_direction_deg=scan_direction_deg,
        scan_lines=tuple(enumerate(line_positions)),
    )


def generate_capture_plan_on_scan_lines(
    effective_geometry: Polygonal,
    *,
    camera: CameraConfig,
    flight_altitude_m: float,
    ground_elevation_m: float,
    scan_direction_deg: float,
    scan_lines: Sequence[tuple[int, float]],
) -> CoveragePlan:
    """Generate lanes on an existing rotated-coordinate scanline lattice."""
    if effective_geometry.is_empty:
        return CoveragePlan(scan_direction_deg % 360.0, (), ())
    if not effective_geometry.is_valid:
        raise ScanlinePlanningError("effective search geometry must be valid")
    dimensions = ground_footprint_dimensions(
        camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
    )
    direction_deg = scan_direction_deg % 360.0
    rotation_deg = direction_deg - 90.0
    scan_geometry = affinity.rotate(effective_geometry, rotation_deg, origin=(0.0, 0.0))
    min_x, _, max_x, _ = scan_geometry.bounds
    margin_m = max(max_x - min_x, dimensions.length_m) + dimensions.length_m

    waypoints: list[Waypoint] = []
    segments: list[ScanSegment] = []
    for line_index, line_y in scan_lines:
        line = LineString([(min_x - margin_m, line_y), (max_x + margin_m, line_y)])
        line_segments = sorted(
            _line_parts(scan_geometry.intersection(line)), key=lambda segment: segment.bounds[0]
        )
        reverse = line_index % 2 == 1
        if reverse:
            line_segments.reverse()
        for segment_index, line_segment in enumerate(line_segments):
            start_x, end_x = line_segment.bounds[0], line_segment.bounds[2]
            sample_x = _capture_positions(
                start_x,
                end_x,
                footprint_length_m=dimensions.length_m,
                spacing_m=dimensions.length_m,
            )
            if reverse:
                sample_x.reverse()
            yaw_deg = direction_deg if not reverse else (direction_deg + 180.0) % 360.0
            ids = []
            segment_points = []
            for x in sample_x:
                scan_point = affinity.rotate(
                    LineString([(x, line_y), (x, line_y)]),
                    -rotation_deg,
                    origin=(0.0, 0.0),
                ).coords[0]
                point = (float(scan_point[0]), float(scan_point[1]))
                waypoint_id = f"wp_{len(waypoints) + 1:04d}"
                ids.append(waypoint_id)
                segment_points.append(point)
                waypoints.append(Waypoint(
                    id=waypoint_id,
                    sequence=len(waypoints) + 1,
                    kind="capture",
                    x=point[0],
                    y=point[1],
                    z=flight_altitude_m,
                    yaw_deg=yaw_deg,
                    camera_pitch_deg=camera.pitch_deg,
                    capture=True,
                    scan_line_index=line_index,
                    scan_segment_index=segment_index,
                    camera_footprint_enu=ground_footprint_polygon(
                        camera,
                        center_enu_m=point,
                        flight_altitude_m=flight_altitude_m,
                        ground_elevation_m=ground_elevation_m,
                        yaw_deg=yaw_deg,
                    ),
                ))
            segments.append(ScanSegment(
                scan_line_index=line_index,
                segment_index=segment_index,
                start_enu_m=segment_points[0],
                end_enu_m=segment_points[-1],
                direction_yaw_deg=yaw_deg,
                capture_waypoint_ids=tuple(ids),
            ))
    return CoveragePlan(direction_deg, tuple(segments), tuple(waypoints))


def scan_line_lattice(
    effective_geometry: Polygonal,
    *,
    camera: CameraConfig,
    flight_altitude_m: float,
    ground_elevation_m: float,
    scan_direction_deg: float,
) -> tuple[tuple[int, float], ...]:
    """Return one shared scanline spacing and phase for a complete geometry."""
    if effective_geometry.is_empty:
        return ()
    dimensions = ground_footprint_dimensions(
        camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
    )
    rotated = affinity.rotate(
        effective_geometry, scan_direction_deg % 360.0 - 90.0, origin=(0.0, 0.0))
    _, min_y, _, max_y = rotated.bounds
    return tuple(enumerate(_scan_line_positions(
        min_y,
        max_y,
        footprint_width_m=dimensions.width_m,
        spacing_m=dimensions.scan_line_spacing_m,
    )))


def _scan_line_positions(
    min_y: float,
    max_y: float,
    *,
    footprint_width_m: float,
    spacing_m: float,
) -> list[float]:
    extent = max_y - min_y
    if extent <= footprint_width_m:
        return [(min_y + max_y) / 2.0]
    first = min_y + footprint_width_m / 2.0
    last = max_y - footprint_width_m / 2.0
    interval_count = _interval_count(last - first, spacing_m)
    actual_spacing = (last - first) / interval_count
    return [first + index * actual_spacing for index in range(interval_count + 1)]


def _capture_positions(
    start_x: float,
    end_x: float,
    *,
    footprint_length_m: float,
    spacing_m: float,
) -> list[float]:
    extent = end_x - start_x
    if extent <= footprint_length_m:
        return [(start_x + end_x) / 2.0]
    first = start_x + footprint_length_m / 2.0
    last = end_x - footprint_length_m / 2.0
    interval_count = _interval_count(last - first, spacing_m)
    actual_spacing = (last - first) / interval_count
    return [first + index * actual_spacing for index in range(interval_count + 1)]


def _interval_count(distance_m: float, maximum_spacing_m: float) -> int:
    """Ceil a spacing ratio without adding an interval for floating-point noise."""
    return max(1, ceil(distance_m / maximum_spacing_m - 1e-12))


def _line_parts(geometry: object) -> list[LineString]:
    if isinstance(geometry, LineString):
        return [geometry] if geometry.length > 0.0 else []
    if isinstance(geometry, MultiLineString):
        return [line for line in geometry.geoms if line.length > 0.0]
    if isinstance(geometry, GeometryCollection):
        return [part for part in geometry.geoms if isinstance(part, LineString) and part.length > 0.0]
    return []
