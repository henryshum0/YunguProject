from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError
import json
from pathlib import Path

import pytest
from shapely.geometry import Point, Polygon

from coverage_planner.geometry import build_effective_search_area
from coverage_planner.io import ConfigError, load_config, parse_config, parse_map_config
from coverage_planner.routing import select_flight_obstacles


def valid_planner_payload() -> dict[str, object]:
    return {
        "schema_version": "1.2",
        "map_file": "map.json",
        "flight": {"altitude_m": 10},
        "camera": {
            "horizontal_fov_deg": 60,
            "vertical_fov_deg": 45,
            "side_overlap": 0.3,
        },
    }


def valid_map_payload() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "origin": {"x": 1, "y": 2},
        "occupied_areas": [
            {"id": "block", "points": [[8, 4], [12, 4], [12, 6], [8, 6]]},
        ],
    }


def test_defaults_are_loaded_without_a_search_area() -> None:
    config = parse_config(valid_planner_payload(), valid_map_payload())
    assert config.frame_id == "map"
    assert config.origin.x == 1.0
    assert config.flight.ground_elevation_m == 0.0
    assert config.flight.horizontal_clearance_m == 3.0
    assert config.camera.target_width_m == 0.0
    assert config.planner.coverage_generation_method == "global_scanline"
    assert config.planner.scan_direction_deg is None
    assert config.planner.minimum_coverage_ratio == 0.99
    assert config.planner.return_to_origin
    assert config.output_topics.waypoints == "/coverage_planner/waypoints"
    assert config.output_topics.markers == "/coverage_planner/markers"
    with pytest.raises(FrozenInstanceError):
        config.frame_id = "odom"  # type: ignore[misc]


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.update({"unexpected": 1}), "unknown fields"),
        (lambda value: value["camera"].update({"pitch_deg": -90}), "unknown fields"),
        (lambda value: value.update({"schema_version": "1.1"}), "schema_version"),
        (lambda value: value.pop("map_file"), "missing required fields"),
        (lambda value: value.update({"origin": {"x": 1, "y": 2}}), "unknown fields"),
        (lambda value: value.update({"search_area": {"points": []}}), "unknown fields"),
        (lambda value: value.update({"output_topics": {
            "waypoints": "waypoints", "markers": "/markers",
        }}), "absolute ROS topic name"),
        (lambda value: value.update({"output_topics": {
            "waypoints": "/same", "markers": "/same",
        }}), "must differ"),
    ],
)
def test_planner_config_is_strict(mutation, message: str) -> None:
    payload = deepcopy(valid_planner_payload())
    mutation(payload)
    with pytest.raises(ConfigError, match=message):
        parse_config(payload, valid_map_payload())


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.update({"unexpected": 1}), "unknown fields"),
        (lambda value: value.update({"schema_version": "2.0"}), "schema_version"),
        (lambda value: value.update({"origin": {"x": True, "y": 0}}), "finite number"),
        (lambda value: value.update({"occupied_areas": [
            {"id": "same", "points": [[1, 1], [2, 1], [2, 2]]},
            {"id": "same", "points": [[3, 1], [4, 1], [4, 2]]},
        ]}), "IDs must be unique"),
    ],
)
def test_map_config_is_strict(mutation, message: str) -> None:
    payload = deepcopy(valid_map_payload())
    mutation(payload)
    with pytest.raises(ConfigError, match=message):
        parse_map_config(payload)


def test_relative_map_file_is_loaded_next_to_planner_file(tmp_path: Path) -> None:
    map_path = tmp_path / "maps" / "mission_map.json"
    map_path.parent.mkdir()
    map_path.write_text(json.dumps(valid_map_payload()), encoding="utf-8")
    planner = valid_planner_payload()
    planner["map_file"] = "maps/mission_map.json"
    planner_path = tmp_path / "planner.json"
    planner_path.write_text(json.dumps(planner), encoding="utf-8")
    config = load_config(planner_path)
    assert config.map_file == str(map_path)
    assert config.origin.y == 2.0
    assert config.occupied_areas[0].id == "block"


def test_map_file_errors_name_the_map_file(tmp_path: Path) -> None:
    planner = valid_planner_payload()
    planner["map_file"] = "missing-map.json"
    planner_path = tmp_path / "planner.json"
    planner_path.write_text(json.dumps(planner), encoding="utf-8")
    with pytest.raises(ConfigError, match="missing-map.json"):
        load_config(planner_path)


def test_overlapping_occupied_areas_are_accepted_and_become_hard_obstacles() -> None:
    map_payload = valid_map_payload()
    map_payload["occupied_areas"] = [
        {"id": "a", "points": [[4, 2], [10, 2], [10, 7], [4, 7]]},
        {"id": "b", "points": [[8, 4], [14, 4], [14, 9], [8, 9]]},
    ]
    config = parse_config(valid_planner_payload(), map_payload)
    points = ((0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0))
    semantic_map = config.to_semantic_map(points)
    effective = build_effective_search_area(semantic_map, Polygon(points))
    obstacles = select_flight_obstacles(
        semantic_map,
        flight_altitude_m=config.flight.altitude_m,
        vertical_clearance_m=0.0,
        horizontal_clearance_m=1.0,
        allow_overflight_above_buildings=False,
    )
    assert obstacles.building_ids == ("a", "b")
    assert not effective.geometry.covers(Point(9, 5))
    assert obstacles.geometry.covers(Point(3.5, 3))
    assert effective.metrics.building_excluded_area_m2 > 0.0


def test_configured_output_topics_are_preserved() -> None:
    planner = valid_planner_payload()
    planner["output_topics"] = {
        "waypoints": "/mission/coverage_waypoints",
        "markers": "/mission/coverage_markers",
    }
    config = parse_config(planner, valid_map_payload())
    assert config.output_topics.waypoints == "/mission/coverage_waypoints"
    assert config.output_topics.markers == "/mission/coverage_markers"
