"""Deterministic scan-direction selection and uncovered-patch supplementation."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot

from coverage_planner.camera import ground_footprint_polygon
from coverage_planner.coverage.evaluation import evaluate_patch_coverage
from coverage_planner.coverage.generators.base import CoverageStructureGenerator
from coverage_planner.coverage.generators.global_scanline import GlobalScanlineGenerator
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.patch import Patch
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.optimization import GreedyLaneRouter, build_lane_routing_problem


@dataclass(frozen=True, slots=True)
class DirectionScore:
    angle_deg: float
    path_length_m: float
    transition_distance_m: float
    turn_count: int
    segment_count: int
    waypoint_count: int

    @property
    def ranking(self) -> tuple[float, float, int, int, int, float]:
        return (self.path_length_m, self.transition_distance_m, self.turn_count,
                self.segment_count, self.waypoint_count, self.angle_deg)


def optimize_scan_direction(
    geometry: Polygonal, *, camera: CameraConfig, flight_altitude_m: float,
    ground_elevation_m: float, candidate_angles_deg: Sequence[float] = (0.0, 90.0),
    generator: CoverageStructureGenerator | None = None,
) -> tuple[CapturePlan, tuple[DirectionScore, ...]]:
    if not candidate_angles_deg:
        raise ValueError("candidate_angles_deg cannot be empty")
    selected_generator: CoverageStructureGenerator = generator or GlobalScanlineGenerator()
    candidates = []
    for angle in candidate_angles_deg:
        plan = selected_generator.generate(
            geometry, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=angle,
        )
        score = _score(plan)
        candidates.append((score, plan))
    candidates.sort(key=lambda item: item[0].ranking)
    return candidates[0][1], tuple(item[0] for item in candidates)


def supplement_uncovered_patches(
    plan: CapturePlan, patches: Sequence[Patch], *, camera: CameraConfig,
    flight_altitude_m: float, ground_elevation_m: float,
    minimum_coverage_ratio: float = 0.99, maximum_passes: int = 2,
) -> tuple[CapturePlan, tuple[Patch, ...]]:
    waypoints = list(plan.capture_waypoints)
    evaluated: tuple[Patch, ...] = tuple(patches)
    for _ in range(maximum_passes + 1):
        footprints = {
            waypoint.id: waypoint.camera_footprint_enu for waypoint in waypoints
            if waypoint.camera_footprint_enu is not None
        }
        evaluated = evaluate_patch_coverage(
            patches, footprints, minimum_coverage_ratio=minimum_coverage_ratio
        )
        uncovered = [patch for patch in evaluated if not patch.covered]
        if not uncovered:
            break
        for patch in uncovered:
            point = patch.geometry.representative_point()
            waypoint_id = f"wp_{len(waypoints) + 1:04d}"
            yaw = plan.scan_direction_deg
            waypoint = Waypoint(
                id=waypoint_id, sequence=len(waypoints) + 1, kind="capture",
                x=point.x, y=point.y, z=flight_altitude_m, yaw_deg=yaw,
                camera_pitch_deg=camera.pitch_deg, capture=True,
                camera_footprint_enu=ground_footprint_polygon(
                    camera, center_enu_m=(point.x, point.y),
                    flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m, yaw_deg=yaw,
                ),
            )
            insertion_index = _cheapest_insertion_index(waypoints, waypoint)
            waypoints.insert(insertion_index, waypoint)
        waypoints = [replace(waypoint, sequence=index, id=f"wp_{index:04d}")
                     for index, waypoint in enumerate(waypoints, start=1)]
    return replace(plan, capture_waypoints=tuple(waypoints)), evaluated


def prepare_lane_route(
    plan: CapturePlan, *, start_enu_m: tuple[float, float], obstacles: Polygonal,
) -> tuple[tuple[Waypoint, ...], tuple[str, ...]]:
    """Compatibility facade for the canonical lane problem and greedy baseline."""
    problem, skipped = build_lane_routing_problem(
        plan, start_enu_m=start_enu_m, obstacles=obstacles)
    solution = GreedyLaneRouter().solve(problem)
    return solution.ordered_waypoints, (*skipped, *solution.skipped_point_ids)


def _cheapest_insertion_index(waypoints: Sequence[Waypoint], candidate: Waypoint) -> int:
    if len(waypoints) < 2:
        return len(waypoints)
    return min(
        range(1, len(waypoints) + 1),
        key=lambda index: (_insertion_cost(waypoints, candidate, index), index),
    )


def _insertion_cost(waypoints: Sequence[Waypoint], candidate: Waypoint, index: int) -> float:
    before = waypoints[index - 1]
    added = hypot(candidate.x - before.x, candidate.y - before.y)
    if index == len(waypoints):
        return added
    after = waypoints[index]
    return (added + hypot(after.x - candidate.x, after.y - candidate.y)
            - hypot(after.x - before.x, after.y - before.y))


def _score(plan: CapturePlan) -> DirectionScore:
    points = plan.capture_waypoints
    path_length = sum(hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(points))
    transition = 0.0
    for a, b in pairwise(points):
        if (a.scan_line_index, a.scan_segment_index) != (b.scan_line_index, b.scan_segment_index):
            transition += hypot(b.x - a.x, b.y - a.y)
    return DirectionScore(
        angle_deg=plan.scan_direction_deg, path_length_m=path_length,
        transition_distance_m=transition, turn_count=max(0, len(plan.scan_segments) - 1),
        segment_count=len(plan.scan_segments), waypoint_count=len(points),
    )
