"""Canonical lane-routing representation shared by heuristic and exact solvers."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot

from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Point

from coverage_planner.coverage.scanlines import CoveragePlan
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint


@dataclass(frozen=True, slots=True)
class CoverageLane:
    """One generated coverage lane available in forward or reverse orientation."""

    id: str
    waypoints: tuple[Waypoint, ...]
    scan_line_index: int | None
    scan_segment_index: int | None
    coverage_cell_index: int | None

    @property
    def orientations(self) -> tuple[tuple[Waypoint, ...], ...]:
        if len(self.waypoints) < 2:
            return (self.waypoints,)
        return (self.waypoints, tuple(reversed(self.waypoints)))

    @property
    def service_length_m(self) -> float:
        return sum(hypot(b.x - a.x, b.y - a.y)
                   for a, b in pairwise(self.waypoints))


@dataclass(frozen=True, slots=True)
class RouteOptimizationProblem:
    """Generated lanes plus start and obstacles for connection optimization."""

    start_enu_m: tuple[float, float]
    coverage_lanes: tuple[CoverageLane, ...]
    obstacles: Polygonal

    @property
    def jobs(self) -> tuple[CoverageLane, ...]:
        return self.coverage_lanes


@dataclass(frozen=True, slots=True)
class OptimizedRoute:
    """Ordered and oriented lanes before connectors become flight waypoints."""

    method: str
    ordered_waypoints: tuple[Waypoint, ...]
    job_order: tuple[str, ...]
    orientation_indices: tuple[int, ...]
    skipped_point_ids: tuple[str, ...]
    transition_cost_m: float
    return_cost_m: float


LaneJob = CoverageLane
LaneRoutingProblem = RouteOptimizationProblem
LaneRoutingSolution = OptimizedRoute


def build_route_optimization_problem(
    plan: CoveragePlan, *, start_enu_m: tuple[float, float], obstacles: Polygonal,
) -> tuple[RouteOptimizationProblem, tuple[str, ...]]:
    """Convert generated lanes into the common connection-optimization input."""
    lanes: list[CoverageLane] = []
    skipped: list[str] = []
    current_key: tuple[int | None, int | None] | None = None
    current: list[Waypoint] = []

    def finish_job() -> None:
        if not current:
            return
        key = (current[0].scan_line_index, current[0].scan_segment_index)
        if len(current) == 1:
            if obstacles.covers(Point(current[0].x, current[0].y)):
                skipped.append(current[0].id)
            else:
                lanes.append(CoverageLane(
                    _lane_id(len(lanes)), (current[0],), *key,
                    current[0].coverage_cell_index))
            return
        source = LineString([(current[0].x, current[0].y), (current[-1].x, current[-1].y)])
        clipped = source.difference(obstacles)
        parts = list(clipped.geoms) if isinstance(clipped, MultiLineString) else [clipped]
        usable = [part for part in parts if isinstance(part, LineString) and part.length > 2e-6]
        if not usable:
            skipped.extend(waypoint.id for waypoint in current)
            return
        for part in usable:
            start = Point(part.coords[0])
            end = Point(part.coords[-1])
            if obstacles.covers(start):
                start = part.interpolate(1e-6)
            if obstacles.covers(end):
                end = part.interpolate(part.length - 1e-6)
            endpoints = tuple(
                _move_waypoint(template, point)
                for template, point in ((current[0], start), (current[-1], end))
            )
            lanes.append(CoverageLane(
                _lane_id(len(lanes)), endpoints, *key,
                current[0].coverage_cell_index))

    for waypoint in plan.capture_waypoints:
        key = (waypoint.scan_line_index, waypoint.scan_segment_index)
        if waypoint.is_completion:
            finish_job()
            current = []
            current_key = None
            if obstacles.covers(Point(waypoint.x, waypoint.y)):
                skipped.append(waypoint.id)
            else:
                lanes.append(CoverageLane(
                    _lane_id(len(lanes)), (waypoint,), None, None,
                    waypoint.coverage_cell_index))
            continue
        if current and key != current_key:
            finish_job()
            current = []
        current_key = key
        current.append(waypoint)
    finish_job()
    return RouteOptimizationProblem(start_enu_m, tuple(lanes), obstacles), tuple(skipped)


build_lane_routing_problem = build_route_optimization_problem


def renumber_waypoints(waypoints: tuple[Waypoint, ...]) -> tuple[Waypoint, ...]:
    return tuple(
        replace(waypoint, id=f"wp_{index:04d}", sequence=index)
        for index, waypoint in enumerate(waypoints, 1)
    )


def _move_waypoint(template: Waypoint, point: Point) -> Waypoint:
    footprint = template.camera_footprint_enu
    if footprint is not None:
        footprint = affinity.translate(
            footprint, xoff=point.x - template.x, yoff=point.y - template.y)
    return replace(
        template, x=float(point.x), y=float(point.y), camera_footprint_enu=footprint)


def _lane_id(index: int) -> str:
    return f"coverage_lane_{index + 1:04d}"
