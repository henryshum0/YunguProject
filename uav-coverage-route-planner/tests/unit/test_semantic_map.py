import json

import pytest
from pydantic import ValidationError

from coverage_planner.io.semantic_map import rectangle_geometry, search_area_geometry
from coverage_planner.models.semantic_map import SemanticMap


def payload() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "world_name": "test_map",
        "coordinate_frame": "ENU",
        "units": "meters",
        "search_area": {"kind": "rectangle", "coords": [[0, 0], [10, 0], [10, 8], [0, 8]]},
        "nodes": [{
            "id": "building_1",
            "properties": {
                "category": "building", "type": "office", "label": "Building 1",
                "passability": "restricted", "visibility": "public",
                "elevation_min_m": 0, "elevation_max_m": 12,
            },
            "shape": {"type": "rectangle", "min_corner": [2, 3], "max_corner": [5, 7]},
        }],
        "metadata": {"ground_truth_excluded": True, "source": "fixture"},
    }


def test_validates_observed_schema_and_building_geometry() -> None:
    semantic_map = SemanticMap.model_validate(payload())
    assert semantic_map.coordinate_frame == "ENU"
    assert len(semantic_map.building_nodes) == 1
    assert rectangle_geometry(semantic_map.building_nodes[0].shape).area == 12
    assert search_area_geometry(semantic_map).area == 80


@pytest.mark.parametrize("mutation", [
    lambda data: data.update({"coordinate_frame": "NED"}),
    lambda data: data["nodes"][0]["shape"].update({"max_corner": [1, 2]}),  # type: ignore[index]
    lambda data: data.update({"unexpected": True}),
])
def test_rejects_invalid_or_unknown_fields(mutation: object) -> None:
    data = json.loads(json.dumps(payload()))
    mutation(data)  # type: ignore[operator]
    with pytest.raises(ValidationError):
        SemanticMap.model_validate(data)
