"""Line-of-sight tests against a collision mesh."""

from __future__ import annotations

from pathlib import Path
import struct

import numpy as np
import pytest

from detection.occlusion import OcclusionMesh, OcclusionMeshError


def wall(height: float = 5.0, x: float = 5.0) -> OcclusionMesh:
    """A vertical quad at ``x``, spanning y in [-5, 5] and z in [0, height]."""
    return OcclusionMesh(np.array([
        [[x, -5.0, 0.0], [x, 5.0, 0.0], [x, 5.0, height]],
        [[x, -5.0, 0.0], [x, 5.0, height], [x, -5.0, height]],
    ]))


def test_segment_through_the_wall_is_blocked() -> None:
    assert wall().segment_blocked((0.0, 0.0, 2.0), (10.0, 0.0, 2.0))


def test_segment_over_and_beside_the_wall_is_clear() -> None:
    mesh = wall(height=5.0)
    assert not mesh.segment_blocked((0.0, 0.0, 8.0), (10.0, 0.0, 8.0))
    assert not mesh.segment_blocked((0.0, 9.0, 2.0), (10.0, 9.0, 2.0))


def test_segment_ending_before_the_wall_is_clear() -> None:
    assert not wall().segment_blocked((0.0, 0.0, 2.0), (4.0, 0.0, 2.0))


def test_hits_at_the_segment_ends_do_not_count_as_occlusion() -> None:
    """A target resting on the ground must not be occluded by the ground itself."""
    mesh = wall(x=10.0)
    assert not mesh.segment_blocked((0.0, 0.0, 2.0), (10.0, 0.0, 2.0))


def test_binary_stl_round_trip_and_origin_shift(tmp_path: Path) -> None:
    triangle = np.array([[0.0, 0.0, 4.0], [1.0, 0.0, 4.0], [0.0, 1.0, 4.0]], dtype=np.float32)
    payload = bytearray(b"\0" * 80 + struct.pack("<I", 1))
    payload += struct.pack("<3f", 0.0, 0.0, 1.0)
    payload += triangle.tobytes()
    payload += struct.pack("<H", 0)
    stl = tmp_path / "mesh.stl"
    stl.write_bytes(bytes(payload))

    mesh = OcclusionMesh.from_binary_stl(stl, origin_offset=(0.0, 0.0, 1.0))
    assert len(mesh) == 1
    # The triangle is at z=4 in mesh coordinates, so at z=3 after the shift.
    assert mesh.segment_blocked((0.2, 0.2, 0.0), (0.2, 0.2, 6.0))
    assert not mesh.segment_blocked((0.2, 0.2, 0.0), (0.2, 0.2, 2.0))


def test_unreadable_and_malformed_meshes_are_reported(tmp_path: Path) -> None:
    with pytest.raises(OcclusionMeshError, match="cannot read"):
        OcclusionMesh.from_binary_stl(tmp_path / "missing.stl")
    short = tmp_path / "short.stl"
    short.write_bytes(b"\0" * 40)
    with pytest.raises(OcclusionMeshError, match="too short"):
        OcclusionMesh.from_binary_stl(short)
    truncated = tmp_path / "truncated.stl"
    truncated.write_bytes(b"\0" * 80 + struct.pack("<I", 5))
    with pytest.raises(OcclusionMeshError, match="declares 5 triangles"):
        OcclusionMesh.from_binary_stl(truncated)
