from __future__ import annotations

from dataclasses import replace
from itertools import pairwise
from math import hypot
from pathlib import Path
from types import SimpleNamespace

import pytest
from shapely.geometry import LineString, Point, Polygon, box

from coverage_planner.camera import ground_footprint_dimensions
from coverage_planner.coverage.generators import (
    BCDGenerator,
    GlobalScanlineGenerator,
    decompose_boustrophedon_cells,
)
from coverage_planner.io import load_config
from coverage_planner.models import CameraConfig
from coverage_planner.routing.visibility import (
    VisibilityRouter,
    _dijkstra,
    shortest_collision_free_path,
)
from coverage_planner.runtime import PlanningFailed, plan_from_config

ROOT = Path(__file__).resolve().parents[2]


def test_camera_footprint_and_lane_spacing() -> None:
    camera = CameraConfig(90.0, 90.0, 0.25)
    dimensions = ground_footprint_dimensions(
        camera, flight_altitude_m=10.0, ground_elevation_m=0.0)
    assert dimensions.width_m == pytest.approx(20.0)
    assert dimensions.length_m == pytest.approx(20.0)
    assert dimensions.scan_line_spacing_m == pytest.approx(15.0)


def test_global_scanline_and_bcd_generate_coverage_structures() -> None:
    camera = CameraConfig(60.0, 45.0, 0.3)
    geometry = Polygon([(0, 0), (40, 0), (40, 30), (25, 30), (25, 12), (0, 12)])
    global_plan = GlobalScanlineGenerator().generate(
        geometry,
        camera=camera,
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    bcd_plan = BCDGenerator().generate(
        geometry,
        camera=camera,
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert global_plan.scan_segments
    assert bcd_plan.scan_segments
    assert decompose_boustrophedon_cells(geometry, scan_direction_deg=90)
    assert all(geometry.covers(Point(point.x, point.y))
               for point in global_plan.capture_waypoints)


def test_heap_dijkstra_uses_deterministic_shortest_path() -> None:
    graph = {
        0: {2: 1.0, 3: 1.0},
        1: {4: 1.0},
        2: {0: 1.0, 4: 1.0},
        3: {0: 1.0, 4: 1.0},
        4: {1: 1.0, 2: 1.0, 3: 1.0},
    }
    assert _dijkstra(graph, 0, 1) == (0, 2, 4, 1)


def test_visibility_route_matches_expected_obstacle_aware_distance() -> None:
    obstacle = box(-1, -1, 1, 1)
    path = shortest_collision_free_path((-3, 0), (3, 0), obstacle)
    assert path == VisibilityRouter(obstacle).shortest_path((-3, 0), (3, 0))
    assert len(path) == 4
    assert sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in pairwise(path)) == pytest.approx(
        2 * 5 ** 0.5 + 2.0)
    assert all(LineString([a, b]).relate(obstacle)[0] == "F" for a, b in pairwise(path))


@pytest.mark.parametrize("method", ["global_scanline", "bcd"])
def test_end_to_end_planning_is_deterministic_and_returns_sparse_closed_route(method: str) -> None:
    config = load_config(ROOT / "config/example_planner.json")
    config = replace(config, planner=replace(
        config.planner,
        coverage_generation_method=method,  # type: ignore[arg-type]
        scan_direction_deg=90.0,
    ))
    first = plan_from_config(config)
    second = plan_from_config(config)
    assert first.planning_route == second.planning_route
    assert first.coverage_requirement_met
    assert first.coverage_generation_method == method
    assert first.planning_route[0].x == first.planning_route[-1].x
    assert first.planning_route[0].y == first.planning_route[-1].y
    assert len(first.planning_route) < len(first.continuous_flight.waypoints)
    assert any(waypoint.capture for waypoint in first.planning_route)
    for left, right in pairwise(first.planning_route):
        assert LineString([(left.x, left.y), (right.x, right.y)]).relate(
            first.obstacles.geometry)[0] == "F"


def test_required_coverage_failure_reports_patch_ids(monkeypatch) -> None:
    config = load_config(ROOT / "config/example_planner.json")
    failed = SimpleNamespace(
        coverage_requirement_met=False,
        unreachable_patch_ids=("patch_001", "patch_009"),
    )
    monkeypatch.setattr("coverage_planner.runtime.CoveragePlanner.plan", lambda *args, **kwargs: failed)
    with pytest.raises(PlanningFailed, match="patch_001, patch_009"):
        plan_from_config(config)
