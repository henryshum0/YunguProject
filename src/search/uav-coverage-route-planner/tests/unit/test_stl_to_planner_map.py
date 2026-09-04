from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import struct

import pytest

from coverage_planner.io import parse_map_config


WORKSPACE = Path(__file__).resolve().parents[5]
UTILITY_PATH = WORKSPACE / "util" / "stl_to_planner_map.py"
converter = None
if UTILITY_PATH.is_file():
    _SPEC = importlib.util.spec_from_file_location("stl_to_planner_map", UTILITY_PATH)
    assert _SPEC is not None and _SPEC.loader is not None
    converter = importlib.util.module_from_spec(_SPEC)
    _SPEC.loader.exec_module(converter)


pytestmark = pytest.mark.skipif(
    converter is None,
    reason="workspace STL conversion utility is not present",
)


def _cube(x: float, y: float, z: float, size: float, height: float):
    bottom = [(x, y, z), (x + size, y, z), (x + size, y + size, z), (x, y + size, z)]
    top = [(px, py, z + height) for px, py, _ in bottom]
    vertices = bottom + top
    faces = (
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    )
    return [tuple(vertices[index] for index in face) for face in faces]


def _write_binary_stl(path: Path, triangles) -> None:
    data = bytearray(80)
    data.extend(struct.pack("<I", len(triangles)))
    for triangle in triangles:
        data.extend(struct.pack("<12fH", 0.0, 0.0, 0.0, *triangle[0], *triangle[1], *triangle[2], 0))
    path.write_bytes(data)


def test_converter_discards_ground_and_writes_valid_map_json(tmp_path: Path) -> None:
    triangles = _cube(0.0, 0.0, 0.0, 2.0, 3.0) + _cube(5.0, 0.0, 0.0, 1.0, 2.0)
    triangles += [((-10.0, -10.0, -0.5), (10.0, -10.0, -0.5), (10.0, 10.0, -0.5)),
                  ((-10.0, -10.0, -0.5), (10.0, 10.0, -0.5), (-10.0, 10.0, -0.5))]
    source = tmp_path / "collider.stl"
    output = tmp_path / "map.json"
    _write_binary_stl(source, triangles)

    assert converter.main([str(source), "--output", str(output)]) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    parsed = parse_map_config(payload)
    assert parsed.origin.x == 0.0
    assert [area.id for area in parsed.occupied_areas] == ["collider_001", "collider_002"]
    assert len(parsed.occupied_areas) == 2


def test_yungu_collider_generates_twenty_five_raised_footprints() -> None:
    source = WORKSPACE / "VisionFlow-PX4/Tools/simulation/gz/worlds/yungu_collider.stl"
    if not source.is_file():
        pytest.skip("Yungu collider STL is not present in this workspace")
    footprints = converter.component_footprints(
        converter.connected_components(converter.read_binary_stl(source)), 0.1)
    assert len(footprints) == 25
