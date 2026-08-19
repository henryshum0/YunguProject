"""Independent two-drone planning over manually assigned responsibility areas."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from shapely.geometry.base import BaseGeometry

from coverage_planner.models import CameraConfig, SemanticMap
from coverage_planner.planner import CoveragePlanner, PlanResult


@dataclass(frozen=True, slots=True)
class DroneAssignment:
    drone_id: str
    search_geometry: BaseGeometry
    start: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DronePlan:
    drone_id: str
    assigned_geometry: BaseGeometry
    result: PlanResult


@dataclass(frozen=True, slots=True)
class MultiDronePlan:
    drones: tuple[DronePlan, DronePlan]


class TwoDroneCoveragePlanner:
    """Plan two disjoint detection responsibilities without temporal deconfliction."""

    def plan(
        self, *, assignments: tuple[DroneAssignment, DroneAssignment],
        semantic_map: SemanticMap, camera: CameraConfig, planner_options: dict[str, Any],
    ) -> MultiDronePlan:
        first, second = assignments
        if first.drone_id == second.drone_id:
            raise ValueError("drone IDs must be unique")
        overlap = first.search_geometry.intersection(second.search_geometry).area
        if overlap > 1e-6:
            raise ValueError(f"drone responsibility areas overlap by {overlap:.3f} square metres")

        def solve(assignment: DroneAssignment) -> DronePlan:
            result = CoveragePlanner().plan(
                semantic_map=semantic_map, search_geometry=assignment.search_geometry,
                camera=camera, start=assignment.start, **planner_options)
            return DronePlan(assignment.drone_id, assignment.search_geometry, result)

        # GEOS overlay operations are intentionally kept in one thread.  Both
        # missions still belong to one planning transaction and neither vehicle
        # is released until both results are complete.
        first_plan = solve(first)
        second_plan = solve(second)
        return MultiDronePlan((first_plan, second_plan))
