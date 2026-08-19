import pytest
from shapely.geometry import MultiPolygon, Point, Polygon, box

from coverage_planner.coverage import generate_capture_plan
from coverage_planner.coverage.generators import (
    BCDGenerator,
    CoverageStructureGenerator,
    GlobalScanlineGenerator,
    ScanlineClippedGenerator,
    build_boustrophedon_planning_cells,
    decompose_boustrophedon_cells,
    merge_small_boustrophedon_cells,
)
from coverage_planner.models.camera import CameraConfig


def camera() -> CameraConfig:
    return CameraConfig.model_validate({
        "image_width_px": 1000,
        "image_height_px": 1000,
        "horizontal_fov_deg": 90,
        "vertical_fov_deg": 90,
        "pitch_deg": -90,
        "yaw_mode": "follow_path",
        "forward_overlap": 0.5,
        "side_overlap": 0.5,
    })


def test_generates_deterministic_boustrophedon_waypoints() -> None:
    plan = generate_capture_plan(
        box(0, 0, 30, 30),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 2
    assert len(plan.capture_waypoints) == 4
    expected_points = [(10, 10), (20, 10), (20, 20), (10, 20)]
    for waypoint, expected in zip(plan.capture_waypoints, expected_points, strict=True):
        assert (waypoint.x, waypoint.y) == pytest.approx(expected)
    assert [waypoint.yaw_deg for waypoint in plan.capture_waypoints] == [90, 90, 270, 270]
    assert [waypoint.id for waypoint in plan.capture_waypoints] == [
        "wp_0001", "wp_0002", "wp_0003", "wp_0004",
    ]


def test_scanline_intersections_split_around_hole() -> None:
    effective = Polygon(
        [(0, 0), (50, 0), (50, 30), (0, 30)],
        holes=[[(20, 2), (30, 2), (30, 28), (20, 28)]],
    )
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 10
    assert {segment.segment_index for segment in plan.scan_segments} == {0, 1}
    for waypoint in plan.capture_waypoints:
        assert effective.covers(box(waypoint.x, waypoint.y, waypoint.x, waypoint.y))
        assert not (20 < waypoint.x < 30)


def test_supports_disconnected_components() -> None:
    effective = MultiPolygon([box(0, 0, 20, 20), box(40, 0, 60, 20)])
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 6
    assert all(waypoint.x <= 20 or waypoint.x >= 40 for waypoint in plan.capture_waypoints)


def test_narrow_region_uses_single_centered_scanline_and_capture() -> None:
    plan = generate_capture_plan(
        box(10, 20, 14, 23),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 1
    assert len(plan.capture_waypoints) == 1
    waypoint = plan.capture_waypoints[0]
    assert (waypoint.x, waypoint.y) == pytest.approx((12, 21.5))


def test_rotated_scan_direction_preserves_capture_points_inside_geometry() -> None:
    effective = box(0, 0, 40, 30)
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=45,
    )
    assert plan.scan_direction_deg == 45
    assert plan.capture_waypoints
    assert all(
        effective.covers(box(waypoint.x, waypoint.y, waypoint.x, waypoint.y))
        for waypoint in plan.capture_waypoints
    )


def test_same_input_produces_identical_plan() -> None:
    arguments = {
        "camera": camera(),
        "flight_altitude_m": 5,
        "ground_elevation_m": 0,
        "scan_direction_deg": 15,
    }
    first = generate_capture_plan(box(-5, -8, 34, 27), **arguments)  # type: ignore[arg-type]
    second = generate_capture_plan(box(-5, -8, 34, 27), **arguments)  # type: ignore[arg-type]
    assert first == second


def test_scanline_generator_preserves_existing_geometry_baseline() -> None:
    generator: CoverageStructureGenerator = ScanlineClippedGenerator()
    arguments = {
        "camera": camera(),
        "flight_altitude_m": 5,
        "ground_elevation_m": 0,
        "scan_direction_deg": 90,
    }
    assert generator.method == "global_scanline"
    assert generator.generate(box(0, 0, 30, 30), **arguments) == generate_capture_plan(
        box(0, 0, 30, 30), **arguments)  # type: ignore[arg-type]


def test_bcd_splits_only_at_sweep_topology_changes() -> None:
    u_shape = Polygon([
        (0, 0), (10, 0), (10, 10), (7, 10),
        (7, 3), (3, 3), (3, 10), (0, 10),
    ])
    cells = decompose_boustrophedon_cells(u_shape, scan_direction_deg=90)
    assert len(cells) == 3
    assert sum(cell.area for cell in cells) == pytest.approx(u_shape.area)
    assert all(cell.intersection(other).area == pytest.approx(0)
               for index, cell in enumerate(cells) for other in cells[index + 1:])


def test_bcd_generator_uses_same_capture_plan_contract() -> None:
    generator: CoverageStructureGenerator = BCDGenerator()
    geometry = box(0, 0, 30, 30).difference(box(12, 8, 18, 22))
    plan = generator.generate(
        geometry, camera=camera(), flight_altitude_m=5,
        ground_elevation_m=0, scan_direction_deg=90)
    assert generator.method == "bcd"
    assert plan.capture_waypoints
    assert all(segment.coverage_cell_index is not None for segment in plan.scan_segments)
    assert all(geometry.covers(Point(waypoint.x, waypoint.y))
               for waypoint in plan.capture_waypoints)
    cells = build_boustrophedon_planning_cells(
        geometry, camera=camera(), flight_altitude_m=5,
        ground_elevation_m=0, scan_direction_deg=90)
    assert {segment.coverage_cell_index for segment in plan.scan_segments} == set(range(len(cells)))
    assert len({
        (segment.scan_line_index, segment.segment_index)
        for segment in plan.scan_segments
    }) == len(plan.scan_segments)
    assert all(
        cells[segment.coverage_cell_index].covers(Point(segment.start_enu_m))
        and cells[segment.coverage_cell_index].covers(Point(segment.end_enu_m))
        for segment in plan.scan_segments
        if segment.coverage_cell_index is not None
    )


def test_bcd_generates_independent_lawnmower_lanes_per_cell() -> None:
    geometry = Polygon([
        (0, 0), (30, 0), (30, 30), (22, 30),
        (22, 10), (8, 10), (8, 30), (0, 30),
    ])
    arguments = {
        "camera": camera(),
        "flight_altitude_m": 5,
        "ground_elevation_m": 0,
        "scan_direction_deg": 90,
    }
    global_plan = GlobalScanlineGenerator().generate(
        geometry, **arguments)  # type: ignore[arg-type]
    bcd_plan = BCDGenerator().generate(
        geometry, **arguments)  # type: ignore[arg-type]

    def lane_geometry(plan: object) -> set[tuple[float, float, float, float]]:
        return {
            (*segment.start_enu_m, *segment.end_enu_m)
            for segment in plan.scan_segments  # type: ignore[attr-defined]
        }

    assert lane_geometry(bcd_plan) != lane_geometry(global_plan)
    assert len({segment.coverage_cell_index for segment in bcd_plan.scan_segments}) > 1
    assert len({segment.scan_line_index for segment in bcd_plan.scan_segments}) == len(
        bcd_plan.scan_segments)


def test_small_bcd_cell_merges_into_adjacent_planning_cell() -> None:
    large = box(0, 0, 20, 20)
    small = box(20, 0, 22, 5)
    cells = merge_small_boustrophedon_cells(
        (large, small), maximum_small_area_m2=20)
    assert len(cells) == 1
    assert cells[0].area == pytest.approx(large.area + small.area)
    assert cells[0].covers(large)
    assert cells[0].covers(small)
