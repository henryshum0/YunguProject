import pytest
from shapely.geometry import LineString, box

from coverage_planner.models.semantic_map import SemanticMap
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.routing import (
    RoutingError,
    route_reachable_waypoints,
    select_flight_obstacles,
    shortest_collision_free_path,
)


def semantic_map() -> SemanticMap:
    def node(identifier: str, x: float, height: float) -> dict[str, object]:
        return {
            "id": identifier,
            "properties": {"category": "building", "type": "building", "label": identifier,
                "passability": "restricted", "visibility": "public",
                "elevation_min_m": 0, "elevation_max_m": height},
            "shape": {"type": "rectangle", "min_corner": [x, 2], "max_corner": [x + 4, 8]},
        }
    return SemanticMap.model_validate({"schema_version": "1.0", "world_name": "routing",
        "coordinate_frame": "ENU", "units": "meters",
        "search_area": {"kind": "rectangle", "coords": [[0, 0], [30, 0], [30, 10], [0, 10]]},
        "nodes": [node("low", 5, 10), node("high", 15, 28)],
        "metadata": {"ground_truth_excluded": True, "source": "fixture"}})


def test_selects_obstacles_by_altitude_and_buffers() -> None:
    obstacles = select_flight_obstacles(
        semantic_map(), flight_altitude_m=30, vertical_clearance_m=2,
        horizontal_clearance_m=3, allow_overflight_above_buildings=True,
    )
    assert obstacles.building_ids == ("high",)
    assert obstacles.geometry.bounds == pytest.approx((12, -1, 22, 11))
    all_obstacles = select_flight_obstacles(
        semantic_map(), flight_altitude_m=100, vertical_clearance_m=2,
        horizontal_clearance_m=0, allow_overflight_above_buildings=False,
    )
    assert all_obstacles.building_ids == ("low", "high")


def test_allows_flight_below_or_above_an_elevated_structure() -> None:
    payload = semantic_map().model_dump()
    payload["nodes"][0]["properties"].update({
        "elevation_min_m": 20, "elevation_max_m": 25, "ground_contact": False})
    elevated = SemanticMap.model_validate(payload)
    below = select_flight_obstacles(
        elevated, flight_altitude_m=17, vertical_clearance_m=2,
        horizontal_clearance_m=0)
    inside = select_flight_obstacles(
        elevated, flight_altitude_m=19, vertical_clearance_m=2,
        horizontal_clearance_m=0)
    above = select_flight_obstacles(
        elevated, flight_altitude_m=28, vertical_clearance_m=2,
        horizontal_clearance_m=0)
    assert "low" not in below.building_ids
    assert "low" in inside.building_ids
    assert "low" not in above.building_ids


def test_uses_visual_safety_override_instead_of_smaller_collision_box() -> None:
    payload = semantic_map().model_dump()
    payload["building_safety_overrides"] = {
        "low": {"min_corner": [3, 1], "max_corner": [10, 9],
                "elevation_min_m": 0, "elevation_max_m": 35}}
    overridden = SemanticMap.model_validate(payload)
    obstacles = select_flight_obstacles(
        overridden, flight_altitude_m=30, vertical_clearance_m=2,
        horizontal_clearance_m=3)
    assert "low" in obstacles.building_ids
    assert obstacles.geometry.covers(box(0, -2, 13, 12))


def test_visibility_graph_routes_around_obstacle() -> None:
    obstacle = box(4, 2, 6, 8)
    points = shortest_collision_free_path((0, 5), (10, 5), obstacle)
    assert len(points) >= 3
    for start, end in pairwise(points):
        assert LineString([start, end]).relate(obstacle)[0] == "F"


def test_direct_route_remains_direct() -> None:
    assert shortest_collision_free_path((0, 0), (10, 0), box(4, 2, 6, 8)) == ((0, 0), (10, 0))


def test_rejects_endpoint_inside_obstacle() -> None:
    with pytest.raises(RoutingError, match="endpoint"):
        shortest_collision_free_path((5, 5), (10, 5), box(4, 2, 6, 8))


def test_routes_reachable_destinations_and_reports_blocked_ones() -> None:
    start = Waypoint("start", 0, "transit", 0, 0, 10, 0, -90, False)
    blocked = Waypoint("blocked", 1, "capture", 5, 5, 10, 0, -90, True)
    reachable = Waypoint("reachable", 2, "capture", 10, 0, 10, 0, -90, True)
    route, skipped = route_reachable_waypoints(start, (blocked, reachable), box(4, 2, 6, 8))
    assert skipped == ("blocked",)
    assert any(waypoint.id == "reachable" for waypoint in route)
    assert all(waypoint.id != "blocked" for waypoint in route)


def test_closed_route_returns_to_start() -> None:
    start = Waypoint("start", 0, "transit", 0, 0, 30, 0, -90, False)
    capture = Waypoint("capture", 1, "capture", 10, 0, 30, 0, -90, True)
    route, skipped = route_reachable_waypoints(
        start, (capture,), box(4, 2, 6, 8), return_to_start=True
    )
    assert skipped == ()
    assert (route[0].x, route[0].y) == (0, 0)
    assert (route[-1].x, route[-1].y) == (0, 0)
    assert route[-1].id == "wp_home_return"
    assert not route[-1].capture


def test_completion_point_preserves_planned_camera_yaw() -> None:
    start = Waypoint("start", 0, "transit", 0, 0, 30, 0, -90, False)
    completion = Waypoint(
        "completion", 1, "capture", 10, 0, 30, 135, -90, True,
        is_completion=True)
    route, skipped = route_reachable_waypoints(start, (completion,), box(4, 2, 6, 8))
    assert skipped == ()
    assert route[-1].yaw_deg == 135
from itertools import pairwise
