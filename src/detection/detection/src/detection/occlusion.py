"""Line-of-sight tests against the world collision mesh.

A target that sits behind a building is not in the camera image, so the mock
detector must not report it: without this check a coverage search could "find" a
target through a wall and the integration logic would be validated against
something the real camera can never see.

The Yungu collision mesh is a 302-triangle STL, so a brute-force segment/triangle
test (Moller-Trumbore, vectorized over all triangles) costs microseconds per
target and needs no acceleration structure.
"""

from __future__ import annotations

from pathlib import Path
import struct

import numpy as np


class OcclusionMeshError(ValueError):
    """Raised when a collision mesh cannot be loaded."""


class OcclusionMesh:
    """Triangles of a world collision mesh, expressed in the ENU world frame."""

    def __init__(self, vertices: np.ndarray) -> None:
        triangles = np.asarray(vertices, dtype=float)
        if triangles.ndim != 3 or triangles.shape[1:] != (3, 3):
            raise OcclusionMeshError("collision mesh must have shape (N, 3, 3)")
        self._origin = triangles[:, 0, :]
        self._edge1 = triangles[:, 1, :] - triangles[:, 0, :]
        self._edge2 = triangles[:, 2, :] - triangles[:, 0, :]

    def __len__(self) -> int:
        return int(self._origin.shape[0])

    @classmethod
    def from_binary_stl(cls, path: str | Path, *, origin_offset=(0.0, 0.0, 0.0)) -> "OcclusionMesh":
        """Load a binary STL and shift it into the ENU world frame.

        ``origin_offset`` is the ENU origin expressed in the mesh's own
        coordinates (for the Gazebo worlds: the airframe spawn pose), so it is
        subtracted from every vertex.
        """
        mesh_path = Path(path).expanduser()
        try:
            payload = mesh_path.read_bytes()
        except OSError as error:
            raise OcclusionMeshError(f"cannot read collision mesh '{mesh_path}': {error}") from error
        if len(payload) < 84:
            raise OcclusionMeshError(f"collision mesh '{mesh_path}' is too short to be a binary STL")
        count = struct.unpack("<I", payload[80:84])[0]
        expected = 84 + count * 50
        if count == 0 or len(payload) < expected:
            raise OcclusionMeshError(
                f"collision mesh '{mesh_path}' declares {count} triangles but holds "
                f"{max(len(payload) - 84, 0)} of {count * 50} bytes")
        records = np.frombuffer(payload[84:expected], dtype=np.uint8).reshape(count, 50)
        # Each 50-byte record is a 12-byte normal, three 12-byte vertices, and a
        # 2-byte attribute count; only the vertices are needed.
        vertices = records[:, 12:48].copy().view(np.float32).reshape(count, 3, 3)
        offset = np.asarray(origin_offset, dtype=float).reshape(3)
        return cls(vertices.astype(float) - offset)

    def segment_blocked(
        self,
        start,
        end,
        *,
        start_margin: float = 1e-3,
        end_margin: float = 1e-2,
    ) -> bool:
        """Return whether any triangle crosses the open segment ``start`` to ``end``.

        The margins exclude hits at the ends of the segment: a ground target
        rests on the ground plane, which is part of the mesh, so a hit at the
        target itself must not count as an occlusion.
        """
        origin = np.asarray(start, dtype=float).reshape(3)
        direction = np.asarray(end, dtype=float).reshape(3) - origin
        pvec = np.cross(direction, self._edge2)
        determinant = np.einsum("ij,ij->i", self._edge1, pvec)
        parallel = np.abs(determinant) < 1e-12
        safe_determinant = np.where(parallel, 1.0, determinant)
        inverse = 1.0 / safe_determinant

        tvec = origin - self._origin
        u = np.einsum("ij,ij->i", tvec, pvec) * inverse
        qvec = np.cross(tvec, self._edge1)
        v = np.einsum("ij,ij->i", np.broadcast_to(direction, tvec.shape), qvec) * inverse
        t = np.einsum("ij,ij->i", self._edge2, qvec) * inverse

        hit = (
            ~parallel
            & (u >= 0.0) & (u <= 1.0)
            & (v >= 0.0) & (u + v <= 1.0)
            & (t > start_margin) & (t < 1.0 - end_margin)
        )
        return bool(hit.any())
