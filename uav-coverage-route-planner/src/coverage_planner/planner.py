"""Pure-Python end-to-end coverage planner orchestration."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot
from typing import Literal

from shapely import unary_union
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from coverage_planner.camera import ground_footprint_dimensions, ground_footprint_polygon
from coverage_planner.coverage import (
    build_continuous_flight_plan,
    evaluate_patch_coverage,
    optimize_scan_direction,
)
from coverage_planner.coverage.generators import BCDGenerator, GlobalScanlineGenerator
from coverage_planner.coverage.generators.base import CoverageStructureGenerator
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.geometry import build_effective_search_area
from coverage_planner.io.semantic_map import building_safety_geometry
from coverage_planner.models import (
    CameraConfig,
    ContinuousFlightPlan,
    Patch,
    PatchGridConfig,
    SemanticMap,
    Waypoint,
)
from coverage_planner.models.search_area import EffectiveSearchArea, Polygonal
from coverage_planner.optimization import (
    OptimizedRoute,
    RouteOptimizationMethod,
    build_route_optimization_problem,
    optimize_route,
)
from coverage_planner.partition import generate_patches
from coverage_planner.routing import (
    FlightObstacles,
    route_reachable_waypoints,
    select_flight_obstacles,
)
from coverage_planner.routing.visibility import RoutingError, VisibilityRouter
from coverage_planner.visibility import visible_detection_ground

_MINIMUM_COMPLETION_SEPARATION_M = 2.0
CompletionStrategy = Literal["full_greedy", "local_insertion"]


@dataclass(frozen=True, slots=True)
class StrategyMetrics:
    pattern: str
    coverage_ratio: float
    planning_point_count: int
    path_length_m: float
    unreachable_patch_count: int


@dataclass(frozen=True, slots=True)
class PatternCandidate:
    pattern: str
    capture_plan: CapturePlan
    planning_route: tuple[Waypoint, ...]
    skipped_point_ids: tuple[str, ...]
    metrics: StrategyMetrics
    route_solution: OptimizedRoute
    route_candidates: tuple[OptimizedRoute, ...]


@dataclass(frozen=True, slots=True)
class UnreachableGround:
    geometry: Polygonal
    area_m2: float
    patch_ids: tuple[str, ...]
    reason: str


@dataclass(frozen=True, slots=True)
class PlanResult:
    semantic_map: SemanticMap
    effective_area: EffectiveSearchArea
    patches: tuple[Patch, ...]
    planning_route: tuple[Waypoint, ...]
    obstacles: FlightObstacles
    scan_direction_deg: float
    unreachable_candidate_point_ids: tuple[str, ...]
    unreachable_patch_ids: tuple[str, ...]
    warnings: tuple[str, ...]
    continuous_flight: ContinuousFlightPlan
    visible_detection_geometry: Polygonal
    visibility_samples: tuple[tuple[str, Polygonal], ...]
    minimum_obstacle_clearance_m: float | None
    minimum_required_coverage_ratio: float
    coverage_requirement_met: bool
    route_optimization_method: str
    route_optimization_candidates: tuple[OptimizedRoute, ...]
    unreachable_ground: tuple[UnreachableGround, ...]
    scan_pattern: str = "global_scanline"

    @property
    def coverage_generation_method(self) -> str:
        """Canonical name for the geometry method; scan_pattern is legacy output."""
        return self.scan_pattern
    strategy_comparison: tuple[StrategyMetrics, ...] = ()
    completion_strategy: CompletionStrategy = "local_insertion"

    @property
    def path_length_m(self) -> float:
        return sum(hypot(b.x-a.x, b.y-a.y) for a, b in pairwise(self.planning_route))


class CoveragePlanner:
    def plan(
        self, *, semantic_map: SemanticMap, search_geometry: BaseGeometry,
        camera: CameraConfig, flight_altitude_m: float, start: tuple[float, float, float],
        horizontal_clearance_m: float = 3.0, vertical_clearance_m: float = 2.0,
        allow_overflight_above_buildings: bool = True, scan_direction_deg: float | None = None,
        patch_config: PatchGridConfig | None = None, ground_elevation_m: float = 0.0,
        minimum_coverage_ratio: float = 0.9999,
        return_to_start: bool = True,
        coverage_generation_method: Literal["global_scanline", "bcd"] | None = None,
        scan_pattern: Literal["scanline_clipped", "bcd", "lawn_mower"] | None = None,
        video_analysis_rate_hz: float = 2.0,
        control_point_spacing_m: float = 10.0,
        coverage_speed_mps: float = 5.0,
        connector_speed_mps: float = 4.0,
        obstacle_speed_mps: float = 2.5,
        return_speed_mps: float = 4.0,
        route_optimization_method: RouteOptimizationMethod = "auto",
        completion_strategy: CompletionStrategy = "local_insertion",
    ) -> PlanResult:
        if completion_strategy not in {"full_greedy", "local_insertion"}:
            raise ValueError(f"unsupported completion strategy: {completion_strategy}")
        effective = build_effective_search_area(semantic_map, search_geometry)
        dimensions = ground_footprint_dimensions(camera, flight_altitude_m=flight_altitude_m,
                                                  ground_elevation_m=ground_elevation_m)
        patches = generate_patches(effective.geometry, config=patch_config, camera_dimensions=dimensions)
        obstacles = select_flight_obstacles(
            semantic_map, flight_altitude_m=flight_altitude_m,
            vertical_clearance_m=vertical_clearance_m,
            horizontal_clearance_m=horizontal_clearance_m,
            allow_overflight_above_buildings=allow_overflight_above_buildings)
        legacy_method = (
            "global_scanline" if scan_pattern in {"lawn_mower", "scanline_clipped"}
            else scan_pattern)
        if (coverage_generation_method is not None and legacy_method is not None
                and coverage_generation_method != legacy_method):
            raise ValueError(
                "coverage_generation_method conflicts with legacy scan_pattern")
        canonical_pattern = coverage_generation_method or legacy_method or "global_scanline"
        patterns = (canonical_pattern,)
        candidates = tuple(self._run_pattern(
            pattern=pattern, effective_geometry=effective.geometry, patches=patches,
            semantic_map=semantic_map,
            camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg,
            minimum_coverage_ratio=minimum_coverage_ratio, start=start,
            obstacle_geometry=obstacles.geometry, return_to_start=return_to_start,
            route_optimization_method=route_optimization_method,
        ) for pattern in patterns)
        best_coverage = max(item.metrics.coverage_ratio for item in candidates)
        competitive = tuple(
            item for item in candidates
            if item.metrics.coverage_ratio >= best_coverage - 0.01
        )
        chosen = min(competitive, key=lambda item: (
            item.metrics.unreachable_patch_count, item.metrics.path_length_m,
            -item.metrics.coverage_ratio, item.pattern))
        capture_plan = chosen.capture_plan
        planning_route = chosen.planning_route
        skipped_point_ids = chosen.skipped_point_ids
        route_solution = chosen.route_solution
        route_candidates = chosen.route_candidates
        service_route = route_solution.ordered_waypoints
        continuous_flight, continuous_footprints = build_continuous_flight_plan(
            planning_route, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            video_analysis_rate_hz=video_analysis_rate_hz,
            control_point_spacing_m=control_point_spacing_m,
            coverage_speed_mps=coverage_speed_mps,
            connector_speed_mps=connector_speed_mps,
            obstacle_speed_mps=obstacle_speed_mps,
            return_speed_mps=return_speed_mps, capture_region=effective.geometry,
            semantic_map=semantic_map)
        evaluated = evaluate_patch_coverage(
            patches, continuous_footprints, minimum_coverage_ratio=minimum_coverage_ratio
        )
        safe_observation_geometry = effective.map_geometry.difference(obstacles.geometry)
        attempted_supplement_points: set[tuple[float, float]] = set()
        completion_visibility_cache: dict[tuple[float, float], BaseGeometry] = {}
        for _ in range(10):
            unresolved = [patch for patch in evaluated if not patch.covered]
            if not unresolved:
                break
            supplemental_waypoints = list(capture_plan.capture_waypoints)
            covered_geometry = (
                unary_union(tuple(continuous_footprints.values()))
                if continuous_footprints else Polygon())
            added = False
            while True:
                uncovered_geometries = tuple(
                    geometry for patch in unresolved
                    if (geometry := self._required_uncovered_geometry(
                        patch, covered_geometry,
                        minimum_coverage_ratio=minimum_coverage_ratio,
                    )) is not None)
                if not uncovered_geometries:
                    break
                total_uncovered = unary_union(uncovered_geometries)
                choices: list[tuple[float, float, float, Point, BaseGeometry]] = []
                candidate_positions: set[tuple[float, float]] = set()
                for uncovered_geometry in uncovered_geometries:
                    for point in self._coverage_completion_candidates(
                            uncovered_geometry,
                            safe_observation_geometry=safe_observation_geometry,
                            camera=camera,
                            flight_altitude_m=flight_altitude_m,
                            ground_elevation_m=ground_elevation_m):
                        position = (round(point.x, 6), round(point.y, 6))
                        known_positions = (
                            attempted_supplement_points | candidate_positions)
                        if (position in known_positions or any(
                                hypot(position[0] - known[0], position[1] - known[1])
                                < _MINIMUM_COMPLETION_SEPARATION_M
                                for known in known_positions)):
                            continue
                        candidate_positions.add(position)
                        visibility = completion_visibility_cache.get(position)
                        if visibility is None:
                            visibility = visible_detection_ground(
                                camera=camera, center_enu_m=(point.x, point.y),
                                flight_altitude_m=flight_altitude_m,
                                ground_elevation_m=ground_elevation_m,
                                yaw_deg=capture_plan.scan_direction_deg,
                                semantic_map=semantic_map,
                            ).intersection(effective.geometry)
                            completion_visibility_cache[position] = visibility
                        gain = visibility.intersection(total_uncovered).area
                        if gain <= 1e-6:
                            continue
                        insertion_cost = self._minimum_insertion_cost(
                            planning_route, point)
                        choices.append((
                            gain / max(1.0, insertion_cost), gain,
                            -insertion_cost, point, visibility))
                if not choices:
                    break
                _, _, _, point, completion_visibility = max(
                    choices, key=lambda item: item[:3])
                attempted_supplement_points.add(
                    (round(point.x, 6), round(point.y, 6)))
                sample_id = f"wp_{len(supplemental_waypoints) + 1:04d}"
                supplemental_waypoints.append(Waypoint(
                    id=sample_id, sequence=len(supplemental_waypoints) + 1,
                    kind="capture", x=point.x, y=point.y, z=flight_altitude_m,
                    yaw_deg=capture_plan.scan_direction_deg,
                    camera_pitch_deg=camera.pitch_deg, capture=True,
                    is_completion=True,
                    camera_footprint_enu=ground_footprint_polygon(
                        camera, center_enu_m=(point.x, point.y),
                        flight_altitude_m=flight_altitude_m,
                        ground_elevation_m=ground_elevation_m,
                        yaw_deg=capture_plan.scan_direction_deg)))
                covered_geometry = unary_union((covered_geometry, completion_visibility))
                added = True
            if not added:
                break
            previous_waypoint_count = len(capture_plan.capture_waypoints)
            capture_plan = replace(
                capture_plan, capture_waypoints=tuple(supplemental_waypoints))
            if completion_strategy == "full_greedy":
                route_points, post_skipped, route_solution, route_candidates = (
                    self._optimize_coverage_route(
                        capture_plan, start_enu_m=(start[0], start[1]),
                        obstacles=obstacles.geometry,
                        method="greedy"))
                service_route = route_points
            else:
                new_completion_points = tuple(
                    supplemental_waypoints[previous_waypoint_count:])
                service_route = self._insert_completion_points_locally(
                    service_route,
                    new_completion_points,
                    start_enu_m=(start[0], start[1]),
                    obstacles=obstacles.geometry,
                    return_to_start=return_to_start,
                )
                route_points = service_route
                post_skipped = ()
            planning_route, post_route_skipped = route_reachable_waypoints(
                Waypoint("wp_start", 0, "transit", *start, 0, -90, False),
                route_points, obstacles.geometry, return_to_start=return_to_start)
            skipped_point_ids = tuple(dict.fromkeys(
                (*skipped_point_ids, *post_skipped, *post_route_skipped)))
            continuous_flight, continuous_footprints = build_continuous_flight_plan(
                planning_route, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                video_analysis_rate_hz=video_analysis_rate_hz,
                control_point_spacing_m=control_point_spacing_m,
                coverage_speed_mps=coverage_speed_mps,
                connector_speed_mps=connector_speed_mps,
                obstacle_speed_mps=obstacle_speed_mps,
                return_speed_mps=return_speed_mps, capture_region=effective.geometry,
                semantic_map=semantic_map)
            evaluated = evaluate_patch_coverage(
                patches, continuous_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
        if completion_strategy == "full_greedy":
            (capture_plan, planning_route, skipped_point_ids, route_solution,
             route_candidates, continuous_flight, continuous_footprints, evaluated) = (
                self._prune_redundant_completion_points(
                    capture_plan,
                    planning_route=planning_route,
                    skipped_point_ids=skipped_point_ids,
                    route_solution=route_solution,
                    route_candidates=route_candidates,
                    continuous_flight=continuous_flight,
                    continuous_footprints=continuous_footprints,
                    evaluated=evaluated,
                    patches=patches,
                    camera=camera,
                    flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m,
                    semantic_map=semantic_map,
                    effective_geometry=effective.geometry,
                    obstacle_geometry=obstacles.geometry,
                    start=start,
                    return_to_start=return_to_start,
                    route_optimization_method=route_optimization_method,
                    video_analysis_rate_hz=video_analysis_rate_hz,
                    control_point_spacing_m=control_point_spacing_m,
                    coverage_speed_mps=coverage_speed_mps,
                    connector_speed_mps=connector_speed_mps,
                    obstacle_speed_mps=obstacle_speed_mps,
                    return_speed_mps=return_speed_mps,
                    minimum_coverage_ratio=minimum_coverage_ratio,
                ))
        for index in range(len(planning_route) - 2, 0, -1):
            previous = planning_route[index - 1]
            tip = planning_route[index]
            following = planning_route[index + 1]
            if completion_strategy == "local_insertion" and tip.capture:
                continue
            detour_length = (
                hypot(tip.x - previous.x, tip.y - previous.y)
                + hypot(following.x - tip.x, following.y - tip.y)
            )
            shortcut_length = hypot(
                following.x - previous.x, following.y - previous.y)
            if detour_length < 4.0 or shortcut_length > detour_length * 0.3:
                continue
            shortcut = LineString([
                (previous.x, previous.y), (following.x, following.y)])
            if shortcut.relate(obstacles.geometry)[0] != "F":
                continue
            candidate_route = planning_route[:index] + planning_route[index + 1:]
            candidate_flight, candidate_footprints = build_continuous_flight_plan(
                candidate_route, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                video_analysis_rate_hz=video_analysis_rate_hz,
                control_point_spacing_m=control_point_spacing_m,
                coverage_speed_mps=coverage_speed_mps,
                connector_speed_mps=connector_speed_mps,
                obstacle_speed_mps=obstacle_speed_mps,
                return_speed_mps=return_speed_mps, capture_region=effective.geometry,
                semantic_map=semantic_map)
            candidate_evaluated = evaluate_patch_coverage(
                patches, candidate_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
            if any(not patch.covered for patch in candidate_evaluated):
                continue
            planning_route = candidate_route
            continuous_flight = candidate_flight
            continuous_footprints = candidate_footprints
            evaluated = candidate_evaluated
        unreachable = tuple(p.id for p in evaluated if not p.covered)
        unreachable_ground = self._unreachable_ground(
            evaluated, continuous_footprints, obstacles.geometry)
        effective_area_m2 = effective.geometry.area
        covered_area_m2 = sum(p.area_m2 * p.coverage_ratio for p in evaluated)
        achieved_coverage_ratio = (
            covered_area_m2 / effective_area_m2 if effective_area_m2 else 0.0)
        coverage_requirement_met = (
            not unreachable
            and achieved_coverage_ratio >= minimum_coverage_ratio)
        warnings = []
        if skipped_point_ids:
            if coverage_requirement_met:
                warnings.append(
                    f"{len(skipped_point_ids)} initial candidate coverage points were "
                    "unreachable at the fixed altitude, but final ground coverage met "
                    "the requirement; these points do not represent uncovered ground")
            else:
                warnings.append(
                    f"{len(skipped_point_ids)} initial candidate coverage points were "
                    "unreachable at the fixed altitude; refer to unreachable_patch_ids "
                    "for ground that remains below the coverage requirement")
        if not coverage_requirement_met:
            unresolved_area_m2 = sum(
                patch.area_m2 * (1.0 - patch.coverage_ratio)
                for patch in evaluated if not patch.covered)
            warnings.append(
                f"{len(unreachable)} search patches remain below the required "
                f"{minimum_coverage_ratio:.4f} coverage, with approximately "
                f"{unresolved_area_m2:.2f} m^2 unresolved; mission is not ready "
                "for execution")
        comparison = tuple(item.metrics for item in sorted(candidates, key=lambda item: item.pattern))
        visible_union = (
            unary_union(tuple(continuous_footprints.values()))
            if continuous_footprints else Polygon())
        visible_union = visible_union.intersection(effective.geometry)
        visible_detection_geometry = (
            visible_union if isinstance(visible_union, (Polygon, MultiPolygon)) else Polygon())
        route_coordinates = [(waypoint.x, waypoint.y) for waypoint in planning_route]
        route_geometry: BaseGeometry = (
            LineString(route_coordinates) if len(route_coordinates) > 1
            else Point(route_coordinates[0]))
        blocked_nodes = tuple(node for node in semantic_map.building_nodes
                              if node.id in obstacles.building_ids)
        minimum_clearance = (
            min(route_geometry.distance(building_safety_geometry(semantic_map, node))
                for node in blocked_nodes)
            if blocked_nodes else None)
        return PlanResult(semantic_map, effective, evaluated, planning_route, obstacles,
                          capture_plan.scan_direction_deg, skipped_point_ids, unreachable,
                          tuple(warnings),
                          continuous_flight, visible_detection_geometry,
                          tuple(continuous_footprints.items()),
                          minimum_clearance,
                          minimum_coverage_ratio, coverage_requirement_met,
                          route_solution.method, route_candidates, unreachable_ground,
                          chosen.pattern, comparison, completion_strategy)

    def _run_pattern(
        self, *, pattern: str, effective_geometry: Polygonal, patches: tuple[Patch, ...],
        semantic_map: SemanticMap,
        camera: CameraConfig, flight_altitude_m: float, ground_elevation_m: float,
        scan_direction_deg: float | None, minimum_coverage_ratio: float,
        start: tuple[float, float, float], obstacle_geometry: Polygonal,
        return_to_start: bool,
        route_optimization_method: RouteOptimizationMethod,
    ) -> PatternCandidate:
        generator: CoverageStructureGenerator
        if pattern == "global_scanline":
            generator = GlobalScanlineGenerator()
        elif pattern == "bcd":
            generator = BCDGenerator()
        else:
            raise ValueError(f"unsupported coverage generator: {pattern}")
        if scan_direction_deg is None:
            capture_plan, _ = optimize_scan_direction(
                effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, generator=generator)
        else:
            capture_plan = generator.generate(
                effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg)
        if pattern == "bcd":
            capture_plan = self._prune_redundant_coverage_lanes(
                capture_plan, patches=patches, camera=camera,
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                semantic_map=semantic_map,
                effective_geometry=effective_geometry,
                minimum_coverage_ratio=minimum_coverage_ratio)
        start_wp = Waypoint("wp_start", 0, "transit", *start, 0, -90, False)
        route_points, pre_skipped, route_solution, route_candidates = (
            self._optimize_coverage_route(
                capture_plan, start_enu_m=(start[0], start[1]),
                obstacles=obstacle_geometry, method=route_optimization_method))
        planning_route, skipped = route_reachable_waypoints(
            start_wp, route_points, obstacle_geometry,
            return_to_start=return_to_start)
        skipped = (*pre_skipped, *skipped)
        footprints = {
            w.id: visible_detection_ground(
                camera=camera, center_enu_m=(w.x, w.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=w.yaw_deg,
                semantic_map=semantic_map,
            ).intersection(effective_geometry)
            for w in capture_plan.capture_waypoints
            if w.camera_footprint_enu is not None
            and not obstacle_geometry.covers(Point(w.x, w.y))
        }
        evaluated = evaluate_patch_coverage(
            patches, footprints, minimum_coverage_ratio=minimum_coverage_ratio)
        effective_area = sum(p.area_m2 for p in patches)
        covered_area = sum(p.area_m2 * p.coverage_ratio for p in evaluated)
        metrics = StrategyMetrics(
            pattern=pattern, coverage_ratio=covered_area / effective_area if effective_area else 0,
            planning_point_count=len(planning_route),
            path_length_m=sum(hypot(b.x-a.x, b.y-a.y)
                              for a, b in pairwise(planning_route)),
            unreachable_patch_count=sum(not p.covered for p in evaluated),
        )
        return PatternCandidate(
            pattern, capture_plan, planning_route, skipped, metrics,
            route_solution, route_candidates)

    @staticmethod
    def _optimize_coverage_route(
        capture_plan: CapturePlan,
        *,
        start_enu_m: tuple[float, float],
        obstacles: Polygonal,
        method: RouteOptimizationMethod,
    ) -> tuple[tuple[Waypoint, ...], tuple[str, ...], OptimizedRoute, tuple[OptimizedRoute, ...]]:
        problem, skipped = build_route_optimization_problem(
            capture_plan, start_enu_m=start_enu_m, obstacles=obstacles)
        solution, candidates = optimize_route(problem, method=method)
        return (
            solution.ordered_waypoints,
            (*skipped, *solution.skipped_point_ids),
            solution,
            candidates,
        )

    @staticmethod
    def _prune_redundant_coverage_lanes(
        capture_plan: CapturePlan,
        *,
        patches: tuple[Patch, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
        minimum_coverage_ratio: float,
    ) -> CapturePlan:
        waypoints = {waypoint.id: waypoint for waypoint in capture_plan.capture_waypoints}
        footprints = {
            waypoint.id: visible_detection_ground(
                camera=camera, center_enu_m=(waypoint.x, waypoint.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=waypoint.yaw_deg,
                semantic_map=semantic_map).intersection(effective_geometry)
            for waypoint in capture_plan.capture_waypoints
        }
        retained = list(capture_plan.scan_segments)
        for segment in sorted(
                capture_plan.scan_segments,
                key=lambda item: (len(item.capture_waypoint_ids), item.scan_line_index,
                                  item.segment_index)):
            if len(segment.capture_waypoint_ids) > 1:
                continue
            candidate_footprints = {
                identifier: geometry for identifier, geometry in footprints.items()
                if identifier not in segment.capture_waypoint_ids}
            evaluated = evaluate_patch_coverage(
                patches, candidate_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
            if not evaluated or any(not patch.covered for patch in evaluated):
                continue
            footprints = candidate_footprints
            retained.remove(segment)
            for identifier in segment.capture_waypoint_ids:
                waypoints.pop(identifier, None)
        ordered = tuple(
            replace(waypoint, id=f"wp_{index:04d}", sequence=index)
            for index, waypoint in enumerate(waypoints.values(), 1))
        id_by_position = {
            (waypoint.scan_line_index, waypoint.scan_segment_index, waypoint.x, waypoint.y): waypoint.id
            for waypoint in ordered}
        segments = tuple(replace(
            segment,
            capture_waypoint_ids=tuple(
                id_by_position[(
                    waypoints[identifier].scan_line_index,
                    waypoints[identifier].scan_segment_index,
                    waypoints[identifier].x,
                    waypoints[identifier].y,
                )]
                for identifier in segment.capture_waypoint_ids),
        ) for segment in retained)
        return replace(capture_plan, scan_segments=segments, capture_waypoints=ordered)

    @staticmethod
    def _unreachable_ground(
        patches: tuple[Patch, ...],
        footprints: dict[str, Polygonal],
        obstacles: Polygonal,
    ) -> tuple[UnreachableGround, ...]:
        covered = unary_union(tuple(footprints.values())) if footprints else Polygon()
        output = []
        for patch in patches:
            if patch.covered:
                continue
            geometry = patch.geometry.difference(covered)
            if geometry.is_empty:
                continue
            polygonal = geometry if isinstance(geometry, (Polygon, MultiPolygon)) else Polygon()
            reason = (
                "flight_obstacle_conflict" if obstacles.intersects(polygonal)
                else "camera_visibility_or_geometry_limit")
            output.append(UnreachableGround(
                polygonal, polygonal.area, (patch.id,), reason))
        return tuple(output)

    @staticmethod
    def _required_uncovered_geometry(
        patch: Patch,
        covered_geometry: BaseGeometry,
        *,
        minimum_coverage_ratio: float,
    ) -> BaseGeometry | None:
        covered_area = patch.geometry.intersection(covered_geometry).area
        if covered_area / patch.area_m2 >= minimum_coverage_ratio:
            return None
        uncovered = patch.geometry.difference(covered_geometry)
        return None if uncovered.is_empty else uncovered

    @staticmethod
    def _coverage_completion_candidates(
        uncovered_geometry: BaseGeometry,
        *,
        safe_observation_geometry: BaseGeometry,
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
    ) -> tuple[Point, ...]:
        if safe_observation_geometry.is_empty:
            return ()
        target = uncovered_geometry.representative_point()
        dimensions = ground_footprint_dimensions(
            camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m)
        half_width = dimensions.width_m / 2.0
        half_length = dimensions.length_m / 2.0
        offsets_x = (-half_width, 0.0, half_width)
        offsets_y = (-half_length, 0.0, half_length)
        candidates = (
            nearest_points(
                Point(target.x + dx, target.y + dy), safe_observation_geometry)[1]
            for dx in offsets_x for dy in offsets_y
        )
        unique = {
            (round(point.x, 6), round(point.y, 6)): point
            for point in candidates if safe_observation_geometry.covers(point)
        }
        return tuple(unique.values())

    @staticmethod
    def _minimum_insertion_cost(
        route: tuple[Waypoint, ...], point: Point,
    ) -> float:
        if not route:
            return 0.0
        if len(route) == 1:
            return 2.0 * hypot(point.x - route[0].x, point.y - route[0].y)
        return min(
            hypot(point.x - left.x, point.y - left.y)
            + hypot(right.x - point.x, right.y - point.y)
            - hypot(right.x - left.x, right.y - left.y)
            for left, right in pairwise(route)
        )

    @classmethod
    def _insert_completion_points_locally(
        cls,
        service_route: tuple[Waypoint, ...],
        completion_points: tuple[Waypoint, ...],
        *,
        start_enu_m: tuple[float, float],
        obstacles: Polygonal,
        return_to_start: bool,
    ) -> tuple[Waypoint, ...]:
        """Insert completion jobs without changing primary lane order or orientation."""
        route = list(service_route)
        router = VisibilityRouter(obstacles)

        def distance(left: tuple[float, float], right: tuple[float, float]) -> float:
            try:
                path = router.shortest_path(left, right)
            except RoutingError:
                return float("inf")
            return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in pairwise(path))

        for completion in completion_points:
            candidate = (completion.x, completion.y)
            choices: list[tuple[float, int]] = []
            for index in cls._completion_insertion_indices(tuple(route)):
                left = (route[index - 1].x, route[index - 1].y) if index else start_enu_m
                left_to_candidate = distance(left, candidate)
                if index < len(route):
                    right = (route[index].x, route[index].y)
                    delta = (
                        left_to_candidate + distance(candidate, right)
                        - distance(left, right)
                    )
                elif return_to_start:
                    delta = (
                        left_to_candidate + distance(candidate, start_enu_m)
                        - distance(left, start_enu_m)
                    )
                else:
                    delta = left_to_candidate
                if delta < float("inf"):
                    choices.append((delta, index))
            if choices:
                _, insertion_index = min(choices, key=lambda item: (item[0], item[1]))
                route.insert(insertion_index, completion)
        return tuple(route)

    @staticmethod
    def _completion_insertion_indices(
        service_route: tuple[Waypoint, ...],
    ) -> tuple[int, ...]:
        """Return job boundaries while keeping each primary lane uninterrupted."""
        indices = [0]
        for index, (left, right) in enumerate(pairwise(service_route), 1):
            same_primary_lane = (
                not left.is_completion
                and not right.is_completion
                and left.scan_line_index is not None
                and left.scan_segment_index is not None
                and left.scan_line_index == right.scan_line_index
                and left.scan_segment_index == right.scan_segment_index
            )
            if not same_primary_lane:
                indices.append(index)
        if not indices or indices[-1] != len(service_route):
            indices.append(len(service_route))
        return tuple(indices)

    @classmethod
    def _coverage_completion_point(
        cls,
        uncovered_geometry: BaseGeometry,
        *,
        safe_observation_geometry: BaseGeometry,
        current_route: tuple[Waypoint, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        yaw_deg: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
        excluded_positions: set[tuple[float, float]] | None = None,
    ) -> Point | None:
        candidates = cls._coverage_completion_candidates(
            uncovered_geometry,
            safe_observation_geometry=safe_observation_geometry,
            camera=camera,
            flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m)

        def score(point: Point) -> tuple[float, float]:
            visible = visible_detection_ground(
                camera=camera, center_enu_m=(point.x, point.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg,
                semantic_map=semantic_map).intersection(effective_geometry)
            gain = visible.intersection(uncovered_geometry).area
            connection = min(
                (hypot(point.x - waypoint.x, point.y - waypoint.y)
                 for waypoint in current_route), default=0.0)
            return gain, -connection

        usable_by_position = {
            (round(point.x, 6), round(point.y, 6)): point
            for point in candidates
        }
        for position in excluded_positions or ():
            usable_by_position.pop(position, None)
        usable = list(usable_by_position.values())
        if not usable:
            return None
        selected = max(usable, key=score)
        return selected if score(selected)[0] > 1e-6 else None

    def _prune_redundant_completion_points(
        self,
        capture_plan: CapturePlan,
        *,
        planning_route: tuple[Waypoint, ...],
        skipped_point_ids: tuple[str, ...],
        route_solution: OptimizedRoute,
        route_candidates: tuple[OptimizedRoute, ...],
        continuous_flight: ContinuousFlightPlan,
        continuous_footprints: dict[str, Polygonal],
        evaluated: tuple[Patch, ...],
        patches: tuple[Patch, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
        obstacle_geometry: Polygonal,
        start: tuple[float, float, float],
        return_to_start: bool,
        route_optimization_method: RouteOptimizationMethod,
        video_analysis_rate_hz: float,
        control_point_spacing_m: float,
        coverage_speed_mps: float,
        connector_speed_mps: float,
        obstacle_speed_mps: float,
        return_speed_mps: float,
        minimum_coverage_ratio: float,
    ) -> tuple[
        CapturePlan, tuple[Waypoint, ...], tuple[str, ...], OptimizedRoute,
        tuple[OptimizedRoute, ...], ContinuousFlightPlan, dict[str, Polygonal], tuple[Patch, ...],
    ]:
        original_state = (
            capture_plan, planning_route, continuous_flight,
            continuous_footprints, evaluated)
        completion_sample_id_by_position = {
            (waypoint.x, waypoint.y): f"segment_point_{index + 1:04d}_image_0000"
            for index, waypoint in enumerate(planning_route)
            if waypoint.is_completion}
        completion_ids = [
            waypoint.id for waypoint in capture_plan.capture_waypoints
            if waypoint.is_completion]
        completion_waypoints = {
            waypoint.id: waypoint for waypoint in capture_plan.capture_waypoints
            if waypoint.is_completion
        }
        completion_visibility = {
            identifier: visible_detection_ground(
                camera=camera, center_enu_m=(waypoint.x, waypoint.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=waypoint.yaw_deg,
                semantic_map=semantic_map,
            ).intersection(effective_geometry)
            for identifier, waypoint in completion_waypoints.items()
        }
        retained_completion_ids = list(completion_ids)
        for identifier in reversed(completion_ids):
            others = [completion_visibility[item]
                      for item in retained_completion_ids if item != identifier]
            if not others:
                continue
            unique_visibility = completion_visibility[identifier].difference(
                unary_union(tuple(others)))
            if unique_visibility.area > 1e-6:
                continue
            retained_completion_ids.remove(identifier)
        retained = set(retained_completion_ids)
        capture_plan = replace(
            capture_plan,
            capture_waypoints=tuple(
                waypoint for waypoint in capture_plan.capture_waypoints
                if not waypoint.is_completion or waypoint.id in retained),
        )
        retained_positions = {
            (waypoint.x, waypoint.y) for waypoint in capture_plan.capture_waypoints
            if waypoint.is_completion}
        continuous_footprints = {
            sample_id: footprint
            for sample_id, footprint in continuous_footprints.items()
            if sample_id not in {
                completion_sample_id_by_position[position]
                for position in completion_sample_id_by_position
                if position not in retained_positions}
        }
        planning_route = tuple(
            waypoint for waypoint in planning_route
            if not waypoint.is_completion or (waypoint.x, waypoint.y) in retained_positions)
        candidate_flight, candidate_footprints = build_continuous_flight_plan(
            planning_route, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            video_analysis_rate_hz=video_analysis_rate_hz,
            control_point_spacing_m=control_point_spacing_m,
            coverage_speed_mps=coverage_speed_mps,
            connector_speed_mps=connector_speed_mps,
            obstacle_speed_mps=obstacle_speed_mps,
            return_speed_mps=return_speed_mps, capture_region=effective_geometry,
            semantic_map=semantic_map)
        candidate_evaluated = evaluate_patch_coverage(
            patches, candidate_footprints,
            minimum_coverage_ratio=minimum_coverage_ratio)
        if all(patch.covered for patch in candidate_evaluated):
            continuous_flight = candidate_flight
            continuous_footprints = candidate_footprints
            evaluated = candidate_evaluated
        else:
            (capture_plan, planning_route, continuous_flight,
             continuous_footprints, evaluated) = original_state
        route_points, candidate_skipped, candidate_solution, candidate_routes = (
            self._optimize_coverage_route(
                capture_plan, start_enu_m=(start[0], start[1]),
                obstacles=obstacle_geometry, method=route_optimization_method))
        candidate_route, route_skipped = route_reachable_waypoints(
            Waypoint("wp_start", 0, "transit", *start, 0, -90, False),
            route_points, obstacle_geometry, return_to_start=return_to_start)
        candidate_flight, candidate_footprints = build_continuous_flight_plan(
            candidate_route, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            video_analysis_rate_hz=video_analysis_rate_hz,
            control_point_spacing_m=control_point_spacing_m,
            coverage_speed_mps=coverage_speed_mps,
            connector_speed_mps=connector_speed_mps,
            obstacle_speed_mps=obstacle_speed_mps,
            return_speed_mps=return_speed_mps, capture_region=effective_geometry,
            semantic_map=semantic_map)
        candidate_evaluated = evaluate_patch_coverage(
            patches, candidate_footprints,
            minimum_coverage_ratio=minimum_coverage_ratio)
        if all(patch.covered for patch in candidate_evaluated):
            planning_route = candidate_route
            skipped_point_ids = tuple(dict.fromkeys(
                (*skipped_point_ids, *candidate_skipped, *route_skipped)))
            route_solution = candidate_solution
            route_candidates = candidate_routes
            continuous_flight = candidate_flight
            continuous_footprints = candidate_footprints
            evaluated = candidate_evaluated
        elif route_optimization_method == "auto":
            route_solution = replace(
                route_solution, method="auto:coverage_feasible_greedy")
            route_candidates = candidate_routes
        return (
            capture_plan, planning_route, skipped_point_ids, route_solution,
            route_candidates, continuous_flight, continuous_footprints, evaluated)
