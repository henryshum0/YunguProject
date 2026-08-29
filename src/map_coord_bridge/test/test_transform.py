"""Unit tests for map_coord_bridge transforms."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # src/map_coord_bridge

from map_coord_bridge.transform import (  # noqa: E402
    to_world, to_map, yaw_from_quat, quat_from_yaw,
)

T = (-153.4, -67.2, -23.85)
START = (153.4, 67.2, 25.0)


def test_to_world_translation_only():
    # start in planner frame should land at the world take-off spot (T+start).
    # The model spawns at the world origin, so the take-off spot is (0, 0, z).
    x, y, z = to_world(*START, 0.0, *T)
    assert abs(x) < 1e-9
    assert abs(y) < 1e-9
    assert abs(z - 1.15) < 1e-9


def test_round_trip_to_world_to_map():
    for yaw in (0.0, 30.0, 90.0, -45.0):
        x, y, z = to_world(10.0, 20.0, 5.0, yaw, *T)
        rx, ry, rz = to_map(x, y, z, yaw, *T)
        assert abs(rx - 10.0) < 1e-9
        assert abs(ry - 20.0) < 1e-9
        assert abs(rz - 5.0) < 1e-9


def test_to_map_with_rotation():
    # A world point due east of the origin maps to +y (north) for yaw 90.
    x, y, z = to_map(10.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0)
    assert abs(x) < 1e-9
    assert abs(y + 10.0) < 1e-9


def test_quat_round_trip():
    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 4):
        qx, qy, qz, qw = quat_from_yaw(yaw)
        assert abs(yaw_from_quat(qx, qy, qz, qw) - yaw) < 1e-12
