import pytest
from shapely.geometry import LineString, MultiPolygon, Polygon, box

from coverage_planner.geometry.search_area import SearchAreaError, build_effective_search_area
from coverage_planner.io.geojson import GeoJSONError, polygonal_geometry_from_geojson
from coverage_planner.models.semantic_map import SemanticMap


def semantic_map() -> SemanticMap:
    return SemanticMap.model_validate({
        "schema_version": "1.0",
        "world_name": "search_test",
        "coordinate_frame": "ENU",
        "units": "meters",
        "search_area": {
            "kind": "rectangle",
            "coords": [[0, 0], [20, 0], [20, 10], [0, 10]],
        },
        "nodes": [
            {
                "id": "building_1",
                "properties": {
                    "category": "building",
                    "type": "office",
                    "label": "Building 1",
                    "passability": "restricted",
                    "visibility": "public",
                    "elevation_min_m": 0,
                    "elevation_max_m": 12,
                },
                "shape": {"type": "rectangle", "min_corner": [4, 2], "max_corner": [8, 8]},
            },
            {
                "id": "road_1",
                "properties": {
                    "category": "trans_facility",
                    "type": "road",
                    "label": "Road",
                    "passability": "open",
                    "visibility": "public",
                    "elevation_min_m": 0,
                    "elevation_max_m": 0,
                },
                "shape": {"type": "rectangle", "min_corner": [10, 0], "max_corner": [12, 10]},
            },
        ],
        "metadata": {"ground_truth_excluded": True, "source": "fixture"},
    })


def test_clips_to_map_and_subtracts_only_buildings() -> None:
    result = build_effective_search_area(semantic_map(), box(-5, 0, 15, 10))
    assert result.metrics.requested_area_m2 == 200
    assert result.metrics.within_map_area_m2 == 150
    assert result.metrics.outside_map_area_m2 == 50
    assert result.metrics.building_excluded_area_m2 == 24
    assert result.metrics.explicit_excluded_area_m2 == 0
    assert result.metrics.effective_search_area_m2 == 126
    assert result.geometry.intersection(box(4, 2, 8, 8)).area == 0
    assert result.geometry.contains(box(10.1, 0.1, 11.9, 9.9))


def test_does_not_remove_ground_below_an_elevated_structure() -> None:
    payload = semantic_map().model_dump()
    payload["nodes"][0]["properties"]["ground_contact"] = False
    result = build_effective_search_area(SemanticMap.model_validate(payload), box(0, 0, 10, 10))
    assert result.geometry.covers(box(4, 2, 8, 8))


def test_subtracts_visual_safety_override_from_search_ground() -> None:
    payload = semantic_map().model_dump()
    payload["building_safety_overrides"] = {
        "building_1": {"min_corner": [3, 1], "max_corner": [9, 9],
                       "elevation_min_m": 0, "elevation_max_m": 15}}
    result = build_effective_search_area(
        SemanticMap.model_validate(payload), box(0, 0, 10, 10))
    assert result.metrics.building_excluded_area_m2 == 48


def test_supports_holes_and_disconnected_multipolygons() -> None:
    polygon_with_hole = Polygon(
        [(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(1, 1), (3, 1), (3, 3), (1, 3)]],
    )
    requested = MultiPolygon([polygon_with_hole, box(15, 0, 18, 3)])
    result = build_effective_search_area(semantic_map(), requested)
    assert isinstance(result.requested_geometry, MultiPolygon)
    assert not result.geometry.contains(box(1.1, 1.1, 2.9, 1.9))
    assert len(result.geometry.geoms) >= 2  # type: ignore[union-attr]


def test_explicit_exclusion_does_not_double_count_building_overlap() -> None:
    result = build_effective_search_area(
        semantic_map(),
        box(0, 0, 10, 10),
        excluded_regions=[box(6, 0, 10, 4)],
    )
    assert result.metrics.building_excluded_area_m2 == 24
    assert result.metrics.explicit_excluded_area_m2 == 12
    assert result.metrics.effective_search_area_m2 == 64
    assert (
        result.metrics.building_excluded_area_m2
        + result.metrics.explicit_excluded_area_m2
        + result.metrics.effective_search_area_m2
        == result.metrics.within_map_area_m2
    )


def test_subtracts_map_authored_excluded_search_regions() -> None:
    payload = semantic_map().model_dump()
    payload["excluded_search_regions"] = [{
        "id": "closed_yard",
        "label": "Closed yard",
        "reason": "No access and no detection responsibility",
        "shape": {"type": "rectangle", "min_corner": [10, 2], "max_corner": [14, 6]},
    }]
    result = build_effective_search_area(
        SemanticMap.model_validate(payload), box(0, 0, 15, 10))
    assert result.metrics.explicit_excluded_area_m2 == 16
    assert result.geometry.intersection(box(10, 2, 14, 6)).area == 0


@pytest.mark.parametrize(
    "geometry, message",
    [
        (Polygon(), "empty"),
        (LineString([(0, 0), (1, 1)]), "Polygon or MultiPolygon"),
        (Polygon([(0, 0), (2, 2), (0, 2), (2, 0)]), "invalid"),
    ],
)
def test_rejects_invalid_search_geometry(geometry: object, message: str) -> None:
    with pytest.raises(SearchAreaError, match=message):
        build_effective_search_area(semantic_map(), geometry)  # type: ignore[arg-type]


def test_reads_geojson_feature_collection_and_preserves_holes() -> None:
    geometry = polygonal_geometry_from_geojson({
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [0, 0], [5, 0], [5, 5], [0, 5], [0, 0],
                    ], [
                        [1, 1], [2, 1], [2, 2], [1, 2], [1, 1],
                    ]],
                },
            },
            {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[[10, 0], [12, 0], [12, 2], [10, 2], [10, 0]]],
                },
            },
        ],
    })
    assert isinstance(geometry, MultiPolygon)
    assert geometry.area == 28


def test_rejects_non_polygonal_geojson() -> None:
    with pytest.raises(GeoJSONError, match="expected Polygon or MultiPolygon"):
        polygonal_geometry_from_geojson({
            "type": "LineString",
            "coordinates": [[0, 0], [1, 1]],
        })
