from pathlib import Path

import pytest
from shapely import unary_union

from coverage_planner.geometry.calibration import MapCalibration
from coverage_planner.geometry.search_area import build_effective_search_area
from coverage_planner.io.geojson import load_polygonal_geojson
from coverage_planner.io.semantic_map import (
    building_safety_geometry,
    load_semantic_map,
    search_area_geometry,
)
from coverage_planner.visualization import render_semantic_map
from coverage_planner.visualization.semantic_map import semantic_map_display_bounds

EXAMPLE = Path("examples/yungu2030/semantic_map.json")


def test_loads_real_yungu_semantic_map() -> None:
    semantic_map = load_semantic_map(EXAMPLE)
    assert semantic_map.world_name == "yungu2030_local_origin"
    assert len(semantic_map.nodes) == 43
    assert len(semantic_map.building_nodes) == 25
    assert search_area_geometry(semantic_map).bounds == (
        -4.835386753082275, -10.610369682312012, 324.72998046875, 210.56312561035156,
    )


def test_builds_real_yungu_outdoor_search_area() -> None:
    semantic_map = load_semantic_map(EXAMPLE)
    requested = load_polygonal_geojson(EXAMPLE.parent / "search_area.geojson")
    result = build_effective_search_area(semantic_map, requested)
    expected_building_area = unary_union([
        building_safety_geometry(semantic_map, node) for node in semantic_map.building_nodes
        if node.properties.ground_contact
    ]).intersection(requested).area
    assert result.metrics.requested_area_m2 == pytest.approx(requested.area)
    assert result.metrics.outside_map_area_m2 == pytest.approx(0)
    assert result.metrics.building_excluded_area_m2 == pytest.approx(expected_building_area)
    assert result.metrics.explicit_excluded_area_m2 > 0
    assert result.metrics.effective_search_area_m2 == pytest.approx(
        requested.area - expected_building_area - result.metrics.explicit_excluded_area_m2)
    assert result.geometry.intersection(result.building_exclusion_geometry).area == pytest.approx(0)


def test_building9_connectors_are_ground_buildings_and_gap_is_not_searchable() -> None:
    semantic_map = load_semantic_map(EXAMPLE)
    connectors = {node.id: node for node in semantic_map.building_nodes
                  if node.id in {"collider_building9.001", "collider_building9.002"}}
    assert set(connectors) == {"collider_building9.001", "collider_building9.002"}
    assert all(node.properties.ground_contact for node in connectors.values())
    assert all(node.properties.elevation_min_m == 0 for node in connectors.values())
    assert [region.id for region in semantic_map.excluded_search_regions] == [
        "no_search_between_building9_connectors"]


def test_renders_real_yungu_map(tmp_path: Path) -> None:
    output = render_semantic_map(load_semantic_map(EXAMPLE), tmp_path / "overview.png")
    assert output.stat().st_size > 10_000


def test_real_overhead_map_calibration_and_overlay(tmp_path: Path) -> None:
    calibration = MapCalibration.load(EXAMPLE.parent / "map_calibration.json")
    assert calibration.pixel_to_enu((269.15, 1026.25)) == pytest.approx((0.0, 0.0), abs=1e-8)
    assert calibration.pixel_to_enu((1678.555, 89.75)) == pytest.approx(
        (305.0, 200.0), abs=1e-8
    )
    assert calibration.content_bounds_enu((1920, 1080)) == pytest.approx(
        (-4.5769314, -10.1975440, 327.3858472, 211.2653497), abs=1e-6)
    assert semantic_map_display_bounds(
        load_semantic_map(EXAMPLE),
        image_size_px=(1920, 1080),
        calibration=calibration,
    ) == pytest.approx((-58.245, -11.265, 357.033, 219.167), abs=0.001)
    output = render_semantic_map(
        load_semantic_map(EXAMPLE),
        tmp_path / "overlay.png",
        background_image_path=EXAMPLE.parent / "overhead_map_rotated_180.jpg",
        calibration=calibration,
    )
    assert output.stat().st_size > 100_000
