"""One-shot planning entry point shared by the ROS node and tests."""

from __future__ import annotations

from shapely.geometry import Polygon

from coverage_planner.models.config import StartupConfig
from coverage_planner.planner import CoveragePlanner, PlanResult


class PlanningFailed(RuntimeError):
    """Raised when planning completes without satisfying required coverage."""


def plan_for_search_area(
    config: StartupConfig,
    search_area_points: tuple[tuple[float, float], ...],
) -> PlanResult:
    """Plan only for a search boundary supplied by a service request."""
    result = CoveragePlanner().plan(
        semantic_map=config.to_semantic_map(search_area_points),
        search_geometry=Polygon(search_area_points),
        camera=config.camera,
        flight_altitude_m=config.flight.altitude_m,
        start=(config.origin.x, config.origin.y, config.flight.altitude_m),
        horizontal_clearance_m=config.flight.horizontal_clearance_m,
        vertical_clearance_m=0.0,
        allow_overflight_above_buildings=False,
        scan_direction_deg=config.planner.scan_direction_deg,
        ground_elevation_m=config.flight.ground_elevation_m,
        minimum_coverage_ratio=config.planner.minimum_coverage_ratio,
        return_to_start=config.planner.return_to_origin,
        coverage_generation_method=config.planner.coverage_generation_method,
        route_optimization_method=config.planner.route_optimization_method,
        completion_strategy=config.planner.completion_strategy,
    )
    if not result.coverage_requirement_met:
        failed = ", ".join(result.unreachable_patch_ids) or "unknown"
        raise PlanningFailed(
            "required coverage was not achieved; failed patch IDs: " + failed)
    return result
