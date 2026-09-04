"""Validated camera configuration used by the coverage geometry."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class CameraConfig:
    """Fixed-nadir camera and conservative target envelope."""

    horizontal_fov_deg: float
    vertical_fov_deg: float
    side_overlap: float
    target_width_m: float = 0.0
    target_length_m: float = 0.0
    target_height_m: float = 0.0
    image_boundary_margin_ratio: float = 0.0
    pitch_deg: float = -90.0

    def __post_init__(self) -> None:
        values = (
            self.horizontal_fov_deg,
            self.vertical_fov_deg,
            self.side_overlap,
            self.target_width_m,
            self.target_length_m,
            self.target_height_m,
            self.image_boundary_margin_ratio,
            self.pitch_deg,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("camera values must be finite numbers")
        if not 0.0 < self.horizontal_fov_deg < 180.0:
            raise ValueError("horizontal_fov_deg must be between 0 and 180")
        if not 0.0 < self.vertical_fov_deg < 180.0:
            raise ValueError("vertical_fov_deg must be between 0 and 180")
        if not 0.0 <= self.side_overlap < 1.0:
            raise ValueError("side_overlap must be in [0, 1)")
        if min(self.target_width_m, self.target_length_m, self.target_height_m) < 0.0:
            raise ValueError("target dimensions cannot be negative")
        if not 0.0 <= self.image_boundary_margin_ratio < 0.5:
            raise ValueError("image_boundary_margin_ratio must be in [0, 0.5)")
        if abs(self.pitch_deg + 90.0) > 1e-9:
            raise ValueError("only a fixed nadir camera (pitch -90 degrees) is supported")
