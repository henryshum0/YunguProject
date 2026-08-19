"""Unit tests for csp_adapter coordinate transform and plan validation."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # src/csp_adapter

from csp_adapter import csp_adapter  # noqa: E402


# --------------------------------------------------------------------------
# to_world
# --------------------------------------------------------------------------

def test_translation_only():
    x, y, z = csp_adapter.to_world(153.4, 67.2, 25.0, 0.0, -157.4, -69.2, -25.0)
    assert x == -4.0
    assert y == -2.0
    assert z == 0.0


def test_rotation_90deg():
    # yaw 90 deg: planner +y (north) maps to world +x (east).
    x, y, z = csp_adapter.to_world(0.0, 10.0, 5.0, 90.0, 1.0, 2.0, 0.0)
    assert abs(x - (1.0 - 10.0)) < 1e-9   # -10 * sin(90) + tx
    assert abs(y - 2.0) < 1e-9            # 0 * cos(90) + ty
    assert z == 5.0


def test_rotation_preserves_length():
    x, y, _ = csp_adapter.to_world(3.0, 4.0, 0.0, 37.0, 0.0, 0.0, 0.0)
    assert abs(math.hypot(x, y) - 5.0) < 1e-9


def test_yaw_to_quat():
    qx, qy, qz, qw = csp_adapter.yaw_to_quat(math.pi / 2.0)
    assert abs(qx) < 1e-12 and abs(qy) < 1e-12
    assert abs(qz - math.sin(math.pi / 4.0)) < 1e-12
    assert abs(qw - math.cos(math.pi / 4.0)) < 1e-12


# --------------------------------------------------------------------------
# load_plan / validation
# --------------------------------------------------------------------------

def _plan(**overrides):
    base = {
        "schema_version": "3.0",
        "coordinate_frame": "ENU",
        "units": "meters",
        "waypoints": [
            {"id": 1, "sequence": 1, "x": 0.0, "y": 0.0, "z": 5.0,
             "heading_deg": 0.0, "speed_mps": 0.0, "turn_in_place": False,
             "hold_time_s": 0.0},
            {"id": 2, "sequence": 2, "x": 10.0, "y": 0.0, "z": 5.0,
             "heading_deg": 90.0, "speed_mps": 3.0, "turn_in_place": False,
             "hold_time_s": 0.0},
        ],
        "route_segments": [
            {"segment_id": 1, "kind": "return_home",
             "start_waypoint_id": 1, "end_waypoint_id": 2,
             "detection_enabled": False},
        ],
        "summary": {"mission_status": "ready"},
    }
    base.update(overrides)
    return base


def test_valid_plan(tmp_path):
    p = tmp_path / "flight_plan.json"
    p.write_text(json.dumps(_plan()))
    plan = csp_adapter.load_plan(str(p))
    assert len(plan["waypoints"]) == 2


def test_rejects_wrong_schema(tmp_path):
    p = tmp_path / "flight_plan.json"
    p.write_text(json.dumps(_plan(schema_version="2.0")))
    try:
        csp_adapter.load_plan(str(p))
        raise AssertionError("expected ValueError")
    except ValueError as exc:
        assert "schema_version" in str(exc)


def test_rejects_non_ready(tmp_path):
    p = tmp_path / "flight_plan.json"
    p.write_text(json.dumps(_plan(summary={"mission_status": "infeasible_coverage"})))
    try:
        csp_adapter.load_plan(str(p))
        raise AssertionError("expected ValueError")
    except ValueError as exc:
        assert "mission_status" in str(exc)


def test_rejects_disconnected_segments(tmp_path):
    bad = _plan(route_segments=[
        {"segment_id": 1, "kind": "coverage_lane",
         "start_waypoint_id": 1, "end_waypoint_id": 2, "detection_enabled": True},
        {"segment_id": 2, "kind": "return_home",
         "start_waypoint_id": 5, "end_waypoint_id": 2, "detection_enabled": False},
    ])
    p = tmp_path / "flight_plan.json"
    p.write_text(json.dumps(bad))
    try:
        csp_adapter.load_plan(str(p))
        raise AssertionError("expected ValueError")
    except ValueError as exc:
        assert "segment 2" in str(exc)
