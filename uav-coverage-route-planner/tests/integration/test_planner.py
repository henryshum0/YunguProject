import json
from pathlib import Path

from shapely import unary_union
from shapely.geometry import LineString, Point, Polygon, box

from coverage_planner.coverage.scanlines import CoveragePlan
from coverage_planner.models import CameraConfig, Patch, SemanticMap, Waypoint
from coverage_planner.planner import CoveragePlanner
from coverage_planner.reporting import export_plan
from coverage_planner.visibility import visible_detection_ground


def semantic_map() -> SemanticMap:
    return SemanticMap.model_validate({"schema_version":"1.0","world_name":"tiny","coordinate_frame":"ENU","units":"meters",
      "search_area":{"kind":"rectangle","coords":[[0,0],[40,0],[40,30],[0,30]]},"nodes":[],
      "metadata":{"ground_truth_excluded":True,"source":"test"}})


def camera() -> CameraConfig:
    return CameraConfig.model_validate({"image_width_px":100,"image_height_px":100,
      "horizontal_fov_deg":90,"vertical_fov_deg":90,"pitch_deg":-90,"yaw_mode":"follow_path",
      "forward_overlap":.5,"side_overlap":.5})


def test_planner_exports_all_required_artifacts(tmp_path: Path) -> None:
    result = CoveragePlanner().plan(semantic_map=semantic_map(), search_geometry=box(0,0,40,30),
      camera=camera(),flight_altitude_m=10,start=(0,0,10),horizontal_clearance_m=1,
      vertical_clearance_m=2,scan_direction_deg=90)
    export_plan(result,tmp_path)
    expected={"patches.geojson","route.geojson","coverage_report.json",
              "visualization.png","flight_plan.json","flight_plan.yaml"}
    assert {path.name for path in tmp_path.iterdir()} == expected
    assert (tmp_path/"visualization.png").stat().st_size>10_000
    assert (result.planning_route[0].x, result.planning_route[0].y) == (
        result.planning_route[-1].x, result.planning_route[-1].y
    )
    assert not result.planning_route[-1].capture
    flight=json.loads((tmp_path/"flight_plan.json").read_text())
    assert flight["schema_version"] == "3.0"
    assert flight["video_detection"]["mode"] == "continuous_video_stream"
    assert flight["video_detection"]["analysis_rate_hz"] == 2
    assert all({"heading_deg", "speed_mps"} <= waypoint.keys()
               for waypoint in flight["waypoints"])
    assert all(segment["speed_mps"] > 0 for segment in flight["route_segments"])
    assert all("source_coverage_cell_index" in segment
               for segment in flight["route_segments"])
    report=json.loads((tmp_path/"coverage_report.json").read_text())
    assert report["optimization_method"] == "coverage_generation_plus_route_optimization"
    assert report["route_optimization_method"].startswith("auto:")
    assert report["route_optimization_candidates"]
    assert report["initial_candidate_metrics"]
    assert report["final_solution_metrics"]["coverage_ratio"] == report["coverage_ratio"]
    assert report["unreachable_candidate_point_count"] == len(
        report["unreachable_candidate_point_ids"])
    assert report["uncovered_patch_count"] == len(report["unreachable_patch_ids"])
    assert "unreachable_ground" in report


def test_planner_is_deterministic() -> None:
    kwargs={"semantic_map":semantic_map(),"search_geometry":box(0,0,40,30),"camera":camera(),
      "flight_altitude_m":10,"start":(0,0,10),"scan_direction_deg":None}
    assert CoveragePlanner().plan(**kwargs)==CoveragePlanner().plan(**kwargs)  # type: ignore[arg-type]


def test_lawn_mower_returns_home_and_avoids_obstacles() -> None:
    result = CoveragePlanner().plan(
        semantic_map=semantic_map(), search_geometry=box(0, 0, 40, 30), camera=camera(),
        flight_altitude_m=10, start=(20, 15, 10), scan_pattern="lawn_mower")
    assert {item.pattern for item in result.strategy_comparison} == {"global_scanline"}
    assert result.coverage_generation_method == "global_scanline"
    assert (result.planning_route[0].x, result.planning_route[0].y) == (20, 15)
    assert (result.planning_route[-1].x, result.planning_route[-1].y) == (20, 15)
    points = {waypoint.id: waypoint for waypoint in result.continuous_flight.waypoints}
    for segment in result.continuous_flight.route_segments:
        start, end = points[segment.start_waypoint_id], points[segment.end_waypoint_id]
        assert LineString([(start.x, start.y), (end.x, end.y)]).relate(
            result.obstacles.geometry)[0] == "F"


def test_bcd_is_a_parallel_coverage_generator() -> None:
    result = CoveragePlanner().plan(
        semantic_map=semantic_map(), search_geometry=box(0, 0, 40, 30), camera=camera(),
        flight_altitude_m=10, start=(20, 15, 10), scan_pattern="bcd",
        scan_direction_deg=90)
    assert result.scan_pattern == "bcd"
    assert result.coverage_requirement_met
    assert result.continuous_flight.lanes
    assert any(segment.source_coverage_cell_index is not None
               for segment in result.continuous_flight.route_segments
               if segment.kind == "coverage_lane")


def test_low_altitude_coverage_uses_only_reachable_captures() -> None:
    map_with_building = SemanticMap.model_validate({"schema_version":"1.0","world_name":"blocked",
      "coordinate_frame":"ENU","units":"meters","search_area":{"kind":"rectangle","coords":[[0,0],[30,0],[30,20],[0,20]]},
      "nodes":[{"id":"building","properties":{"category":"building","type":"office","label":"building",
      "passability":"restricted","visibility":"public","elevation_min_m":0,"elevation_max_m":20},
      "shape":{"type":"rectangle","min_corner":[10,5],"max_corner":[20,15]}}],
      "metadata":{"ground_truth_excluded":True,"source":"test"}})
    result = CoveragePlanner().plan(semantic_map=map_with_building,search_geometry=box(0,0,30,20),
      camera=camera(),flight_altitude_m=10,start=(0,0,10),horizontal_clearance_m=3,
      vertical_clearance_m=2,scan_direction_deg=90)
    assert all(
        contributor.startswith("segment_")
        for patch in result.patches for contributor in patch.covered_by_waypoint_ids)


def test_completion_observation_point_stays_in_safe_free_ground() -> None:
    target = box(10.1, 8, 10.5, 9)
    safe_ground = box(12, 0, 30, 20)
    selected = CoveragePlanner._coverage_completion_point(
        target,
        safe_observation_geometry=safe_ground,
        current_route=(),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        yaw_deg=90,
        semantic_map=semantic_map(),
        effective_geometry=box(0, 0, 40, 30),
    )
    assert isinstance(selected, Point)
    assert safe_ground.covers(selected)
    assert selected.x >= 12


def test_completion_observation_can_stand_outside_responsibility_area() -> None:
    responsibility = box(0, 0, 2, 10)
    selected = CoveragePlanner._coverage_completion_point(
        box(0, 4, .2, 6),
        safe_observation_geometry=box(-10, 0, -1, 10),
        current_route=(),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        yaw_deg=90,
        semantic_map=semantic_map(),
        effective_geometry=responsibility,
    )
    assert isinstance(selected, Point)
    assert not responsibility.covers(selected)
    assert selected.x < 0


def test_completion_search_can_skip_an_already_attempted_position() -> None:
    arguments = {
        "safe_observation_geometry": box(-10, 0, 10, 10),
        "current_route": (),
        "camera": camera(),
        "flight_altitude_m": 10,
        "ground_elevation_m": 0,
        "yaw_deg": 90,
        "semantic_map": semantic_map(),
        "effective_geometry": box(0, 0, 2, 10),
    }
    first = CoveragePlanner._coverage_completion_point(
        box(0, 4, .2, 6), **arguments)
    assert isinstance(first, Point)
    second = CoveragePlanner._coverage_completion_point(
        box(0, 4, .2, 6),
        excluded_positions={(round(first.x, 6), round(first.y, 6))},
        **arguments,
    )
    assert isinstance(second, Point)
    assert (second.x, second.y) != (first.x, first.y)


def test_one_completion_footprint_suppresses_adjacent_patch_candidate() -> None:
    first_patch = box(0, 0, 1, 1)
    adjacent_patch = box(1, 0, 2, 1)
    selected = CoveragePlanner._coverage_completion_point(
        first_patch,
        safe_observation_geometry=box(-10, -10, 10, 10),
        current_route=(), camera=camera(), flight_altitude_m=10,
        ground_elevation_m=0, yaw_deg=90, semantic_map=semantic_map(),
        effective_geometry=box(0, 0, 2, 1),
    )
    assert isinstance(selected, Point)
    visibility = visible_detection_ground(
        camera=camera(), center_enu_m=(selected.x, selected.y),
        flight_altitude_m=10, ground_elevation_m=0, yaw_deg=90,
        semantic_map=semantic_map(),
    ).intersection(box(0, 0, 2, 1))
    covered_geometry = unary_union((Polygon(), visibility))
    assert adjacent_patch.difference(covered_geometry).is_empty


def test_completion_insertion_cost_prefers_candidate_near_existing_route() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 10, 0, -90, True),
        Waypoint("b", 2, "capture", 0, 10, 10, 0, -90, True),
    )
    near = CoveragePlanner._minimum_insertion_cost(route, Point(1, 5))
    far = CoveragePlanner._minimum_insertion_cost(route, Point(20, 5))
    assert near < far


def test_local_completion_insertion_preserves_primary_lane_order() -> None:
    lane_a = (
        Waypoint("a1", 1, "capture", 0, 0, 10, 90, -90, True, 0, 0),
        Waypoint("a2", 2, "capture", 10, 0, 10, 90, -90, True, 0, 0),
    )
    lane_b = (
        Waypoint("b1", 3, "capture", 10, 10, 10, 270, -90, True, 1, 0),
        Waypoint("b2", 4, "capture", 0, 10, 10, 270, -90, True, 1, 0),
    )
    completion = Waypoint(
        "completion", 5, "capture", 5, 1, 10, 90, -90, True,
        is_completion=True,
    )
    route = CoveragePlanner._insert_completion_points_locally(
        lane_a + lane_b, (completion,), start_enu_m=(-5, 0),
        obstacles=Polygon(), return_to_start=True,
    )
    primary_ids = [waypoint.id for waypoint in route if not waypoint.is_completion]
    assert primary_ids == ["a1", "a2", "b1", "b2"]
    assert route.index(lane_a[1]) == route.index(lane_a[0]) + 1
    assert route.index(lane_b[1]) == route.index(lane_b[0]) + 1


def test_local_completion_insertion_uses_obstacle_aware_cost() -> None:
    service_route = (
        Waypoint("a", 1, "capture", 0, 0, 10, 0, -90, True),
        Waypoint("b", 2, "capture", 20, 0, 10, 0, -90, True),
    )
    completion = Waypoint(
        "completion", 3, "capture", 10, 7, 10, 0, -90, True,
        is_completion=True,
    )
    route = CoveragePlanner._insert_completion_points_locally(
        service_route, (completion,), start_enu_m=(-10, 0),
        obstacles=box(8, 1, 12, 6), return_to_start=False,
    )
    assert [waypoint.id for waypoint in route] == ["a", "completion", "b"]


def test_completion_ignores_residual_inside_patch_tolerance() -> None:
    geometry = box(0, 0, 100, 100)
    patch = Patch("patch", 0, 0, (50, 50), geometry, geometry.area)
    allowed_residual = box(99.995, 0, 100, 100)
    covered = geometry.difference(allowed_residual)
    assert CoveragePlanner._required_uncovered_geometry(
        patch, covered, minimum_coverage_ratio=.9999) is None

    excessive_residual = box(99.98, 0, 100, 100)
    covered = geometry.difference(excessive_residual)
    assert CoveragePlanner._required_uncovered_geometry(
        patch, covered, minimum_coverage_ratio=.9999) is not None


def test_many_completion_points_still_receive_final_auto_optimization() -> None:
    map_geometry = box(0, 0, 140, 20)
    many_points = tuple(
        Waypoint(
            f"completion_{index:02d}", index, "capture", index * 10, 10, 10,
            90, -90, True, is_completion=True,
        )
        for index in range(1, 14)
    )
    _route, _skipped, solution, candidates = CoveragePlanner._optimize_coverage_route(
        CoveragePlan(90, (), many_points), start_enu_m=(0, 10),
        obstacles=Polygon(), method="auto",
    )
    assert solution.method == "auto:heuristic"
    assert {candidate.method for candidate in candidates} >= {
        "greedy_obstacle_distance", "two_opt", "or_opt",
    }
    assert map_geometry.covers(LineString(
        (waypoint.x, waypoint.y) for waypoint in solution.ordered_waypoints))
