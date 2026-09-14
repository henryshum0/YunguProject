"""Pinhole camera geometry for the forward-looking detection camera.

Coordinate conventions, all right-handed:

``world``
    ENU, anchored at the drone launch point: x east, y north, z up. This is the
    frame of navigation waypoints and coverage search areas.
``body``
    Gazebo/ROS FLU body frame: x forward, y left, z up.
``camera``
    The Gazebo camera link frame: the optical axis is +X, +Y points image-left
    and +Z points image-up. Image coordinates therefore run *against* y and z::

        u = cx - fx * y_cam / x_cam
        v = cy - fy * z_cam / x_cam

Nothing here is simulation-specific: on the real vehicle the same model is used
with calibrated intrinsics and extrinsics.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, radians, sin, tan

import numpy as np


class CameraGeometryError(ValueError):
    """Raised when a camera model or pose cannot describe a valid projection."""


@dataclass(frozen=True, slots=True)
class PinholeCamera:
    """Intrinsics of a square-pixel pinhole camera plus its far clip."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    max_range_m: float
    frame_id: str = "camera_link"

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise CameraGeometryError("camera width and height must be positive")
        values = (self.fx, self.fy, self.cx, self.cy, self.max_range_m)
        if not all(isfinite(value) for value in values):
            raise CameraGeometryError("camera intrinsics must be finite numbers")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise CameraGeometryError("camera focal lengths must be positive")
        if self.max_range_m <= 0.0:
            raise CameraGeometryError("camera max_range_m must be positive")

    @classmethod
    def from_horizontal_fov(
        cls,
        *,
        width: int,
        height: int,
        horizontal_fov_deg: float,
        max_range_m: float,
        frame_id: str = "camera_link",
    ) -> "PinholeCamera":
        """Build the intrinsics a Gazebo camera sensor produces from its FoV.

        Gazebo uses one focal length for both axes, so the vertical field of
        view follows from the image aspect ratio rather than being independent.
        """
        if not isfinite(horizontal_fov_deg) or not 0.0 < horizontal_fov_deg < 180.0:
            raise CameraGeometryError("horizontal_fov_deg must be between 0 and 180")
        focal_length = (width / 2.0) / tan(radians(horizontal_fov_deg) / 2.0)
        return cls(
            width=int(width),
            height=int(height),
            fx=focal_length,
            fy=focal_length,
            cx=width / 2.0,
            cy=height / 2.0,
            max_range_m=float(max_range_m),
            frame_id=frame_id,
        )

    @property
    def vertical_fov_deg(self) -> float:
        """Vertical field of view implied by the image height and focal length."""
        return 2.0 * np.degrees(np.arctan((self.height / 2.0) / self.fy))

    def project(self, points_camera: np.ndarray) -> np.ndarray:
        """Project camera-frame points to pixels, without any clipping.

        ``points_camera`` is an ``(N, 3)`` array. Points at or behind the image
        plane (``x <= 0``) have no projection and are returned as NaN.
        """
        points = np.atleast_2d(np.asarray(points_camera, dtype=float))
        if points.ndim != 2 or points.shape[1] != 3:
            raise CameraGeometryError("points_camera must have shape (N, 3)")
        depth = points[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.cx - self.fx * points[:, 1] / depth
            v = self.cy - self.fy * points[:, 2] / depth
        pixels = np.stack((u, v), axis=1)
        pixels[depth <= 0.0] = np.nan
        return pixels

    def ray_direction(self, u: float, v: float) -> np.ndarray:
        """Return the unit camera-frame direction looking through pixel ``(u, v)``."""
        direction = np.array([1.0, (self.cx - u) / self.fx, (self.cy - v) / self.fy])
        return direction / np.linalg.norm(direction)

    def image_overlap_fraction(self, box: "PixelBox") -> float:
        """Fraction of ``box`` that falls inside the image rectangle."""
        area = box.area
        if area <= 0.0:
            return 0.0
        overlap_x = max(0.0, min(box.max_x, float(self.width)) - max(box.min_x, 0.0))
        overlap_y = max(0.0, min(box.max_y, float(self.height)) - max(box.min_y, 0.0))
        return (overlap_x * overlap_y) / area

    def clip(self, box: "PixelBox") -> "PixelBox":
        """Clip a box to the image rectangle."""
        return PixelBox(
            min_x=min(max(box.min_x, 0.0), float(self.width)),
            min_y=min(max(box.min_y, 0.0), float(self.height)),
            max_x=max(min(box.max_x, float(self.width)), 0.0),
            max_y=max(min(box.max_y, float(self.height)), 0.0),
        )


@dataclass(frozen=True, slots=True)
class PixelBox:
    """An axis-aligned image-plane box in pixels."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @classmethod
    def from_center(cls, center_x: float, center_y: float, size_x: float, size_y: float) -> "PixelBox":
        half_x = size_x / 2.0
        half_y = size_y / 2.0
        return cls(center_x - half_x, center_y - half_y, center_x + half_x, center_y + half_y)

    @classmethod
    def bounding(cls, pixels: np.ndarray) -> "PixelBox":
        """Axis-aligned bounds of a set of projected pixels."""
        points = np.atleast_2d(np.asarray(pixels, dtype=float))
        if points.size == 0 or not np.isfinite(points).all():
            raise CameraGeometryError("cannot bound a box from non-finite pixels")
        return cls(
            min_x=float(points[:, 0].min()),
            min_y=float(points[:, 1].min()),
            max_x=float(points[:, 0].max()),
            max_y=float(points[:, 1].max()),
        )

    @property
    def size_x(self) -> float:
        return self.max_x - self.min_x

    @property
    def size_y(self) -> float:
        return self.max_y - self.min_y

    @property
    def center_x(self) -> float:
        return (self.min_x + self.max_x) / 2.0

    @property
    def center_y(self) -> float:
        return (self.min_y + self.max_y) / 2.0

    @property
    def short_side(self) -> float:
        """Shorter box side in pixels: what decides whether a detector resolves it."""
        return min(self.size_x, self.size_y)

    @property
    def area(self) -> float:
        return max(self.size_x, 0.0) * max(self.size_y, 0.0)


@dataclass(frozen=True, slots=True)
class Extrinsics:
    """A rigid transform, stored as the child frame's pose in its parent."""

    translation: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        if self.translation.shape != (3,):
            raise CameraGeometryError("extrinsic translation must have shape (3,)")
        if self.rotation.shape != (3, 3):
            raise CameraGeometryError("extrinsic rotation must have shape (3, 3)")

    @classmethod
    def from_rpy(cls, translation, roll_deg: float, pitch_deg: float, yaw_deg: float) -> "Extrinsics":
        """Build a transform from an SDF-style translation and roll/pitch/yaw."""
        offset = np.asarray(translation, dtype=float).reshape(3)
        return cls(translation=offset, rotation=rotation_from_rpy(
            radians(roll_deg), radians(pitch_deg), radians(yaw_deg)))

    @classmethod
    def from_quaternion(cls, translation, quaternion) -> "Extrinsics":
        """Build a transform from a translation and a ``(w, x, y, z)`` quaternion."""
        offset = np.asarray(translation, dtype=float).reshape(3)
        return cls(translation=offset, rotation=rotation_from_quaternion(quaternion))

    def to_child(self, points_parent: np.ndarray) -> np.ndarray:
        """Express parent-frame points in the child frame."""
        points = np.atleast_2d(np.asarray(points_parent, dtype=float))
        return (points - self.translation) @ self.rotation

    def to_parent(self, points_child: np.ndarray) -> np.ndarray:
        """Express child-frame points in the parent frame."""
        points = np.atleast_2d(np.asarray(points_child, dtype=float))
        return points @ self.rotation.T + self.translation

    def direction_to_parent(self, direction_child: np.ndarray) -> np.ndarray:
        """Rotate a child-frame direction into the parent frame."""
        return self.rotation @ np.asarray(direction_child, dtype=float).reshape(3)

    def compose(self, child: "Extrinsics") -> "Extrinsics":
        """Return this transform followed by ``child`` (parent <- self <- child)."""
        return Extrinsics(
            translation=self.translation + self.rotation @ child.translation,
            rotation=self.rotation @ child.rotation,
        )


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix for the SDF/URDF intrinsic roll-pitch-yaw convention."""
    cos_r, sin_r = cos(roll), sin(roll)
    cos_p, sin_p = cos(pitch), sin(pitch)
    cos_y, sin_y = cos(yaw), sin(yaw)
    return np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r],
    ])


def rotation_from_quaternion(quaternion) -> np.ndarray:
    """Rotation matrix for a ``(w, x, y, z)`` quaternion."""
    values = np.asarray(quaternion, dtype=float).reshape(4)
    norm = float(np.linalg.norm(values))
    if not isfinite(norm) or norm <= 0.0:
        raise CameraGeometryError("quaternion must be finite and non-zero")
    w, x, y, z = values / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def box_corners(center, size, yaw_deg: float) -> np.ndarray:
    """Return the eight corners of a yaw-rotated box, as an ``(8, 3)`` array.

    ``size`` is ``(length along the box's own +X, width, height)`` and ``yaw_deg``
    rotates that +X axis counter-clockwise from east.
    """
    middle = np.asarray(center, dtype=float).reshape(3)
    half = np.asarray(size, dtype=float).reshape(3) / 2.0
    if not np.isfinite(middle).all() or not np.isfinite(half).all() or (half <= 0.0).any():
        raise CameraGeometryError("box center must be finite and size must be positive")
    signs = np.array([
        [sx, sy, sz]
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ])
    local = signs * half
    rotation = rotation_from_rpy(0.0, 0.0, radians(yaw_deg))
    return local @ rotation.T + middle
