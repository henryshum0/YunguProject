from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from gui.map_view import (
    Bounds,
    MapLoadError,
    bounds_for,
    load_planner_map,
    make_viewport,
    rectangle_from_clicks,
    route_points,
)


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
YUNGU_MAP = WORKSPACE_ROOT / "src" / "search" / "config" / "yungu_map.json"


def test_yungu_map_loads_all_occupied_areas() -> None:
    map_data = load_planner_map(YUNGU_MAP)
    assert map_data.origin == (0.0, 0.0)
    assert len(map_data.occupied_areas) == 25
    assert map_data.occupied_areas[0].identifier == "collider_001"


def test_map_loader_rejects_invalid_schema_and_polygons(tmp_path: Path) -> None:
    bad_schema = tmp_path / "bad_schema.json"
    bad_schema.write_text(json.dumps({"schema_version": "2.0", "origin": {"x": 0, "y": 0}}))
    with pytest.raises(MapLoadError, match="schema_version"):
        load_planner_map(bad_schema)

    self_intersecting = tmp_path / "bow_tie.json"
    self_intersecting.write_text(json.dumps({
        "schema_version": "1.0",
        "origin": {"x": 0, "y": 0},
        "occupied_areas": [{
            "id": "bad",
            "points": [[0, 0], [2, 2], [0, 2], [2, 0]],
        }],
    }))
    with pytest.raises(MapLoadError, match="valid simple polygon"):
        load_planner_map(self_intersecting)


def test_two_click_rectangle_is_ordered_for_the_planner_service() -> None:
    assert rectangle_from_clicks((8.0, 5.0), (-2.0, 12.0)) == (
        (-2.0, 5.0), (8.0, 5.0), (8.0, 12.0), (-2.0, 12.0))
    with pytest.raises(ValueError, match="non-zero-area"):
        rectangle_from_clicks((0.0, 0.0), (0.0, 4.0))


def test_viewport_preserves_enu_axes_and_round_trips() -> None:
    viewport = make_viewport(Bounds(-10.0, 10.0, -5.0, 5.0), 400.0, 300.0)
    east_north = viewport.to_canvas((10.0, 5.0))
    west_south = viewport.to_canvas((-10.0, -5.0))
    assert east_north[0] > west_south[0]
    assert east_north[1] < west_south[1]
    assert viewport.to_enu(viewport.to_canvas((3.5, -2.0))) == pytest.approx((3.5, -2.0))


def test_bounds_and_route_overlay_points_include_ros_path_coordinates() -> None:
    map_data = load_planner_map(YUNGU_MAP)
    path = SimpleNamespace(poses=[
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=200.0, y=-100.0))),
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=220.0, y=-80.0))),
    ])
    route = route_points(path)
    bounds = bounds_for(map_data, route)
    assert route == ((200.0, -100.0), (220.0, -80.0))
    assert bounds.max_x == 220.0
