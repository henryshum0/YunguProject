"""Validated configuration for the first-version nadir camera model."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class CameraConfig(BaseModel):
    """Fixed nadir camera and requested image overlap."""

    model_config = ConfigDict(extra="forbid")

    image_width_px: int = Field(gt=0)
    image_height_px: int = Field(gt=0)
    horizontal_fov_deg: float = Field(gt=0.0, lt=180.0)
    vertical_fov_deg: float = Field(gt=0.0, lt=180.0)
    pitch_deg: float = -90.0
    yaw_mode: Literal["follow_path", "fixed"] = "follow_path"
    fixed_yaw_deg: float | None = None
    forward_overlap: float = Field(ge=0.0, lt=1.0)
    side_overlap: float = Field(ge=0.0, lt=1.0)
    target_width_m: float = Field(default=0.0, ge=0.0)
    target_length_m: float = Field(default=0.0, ge=0.0)
    target_height_m: float = Field(default=0.0, ge=0.0)
    image_boundary_margin_ratio: float = Field(default=0.0, ge=0.0, lt=0.5)
    minimum_ground_sampling_distance_cm_per_px: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_supported_projection(self) -> CameraConfig:
        if abs(self.pitch_deg + 90.0) > 1e-9:
            raise ValueError("oblique camera projection not implemented; pitch_deg must be -90")
        if self.yaw_mode == "fixed" and self.fixed_yaw_deg is None:
            raise ValueError("fixed_yaw_deg is required when yaw_mode is fixed")
        if self.yaw_mode == "follow_path" and self.fixed_yaw_deg is not None:
            raise ValueError("fixed_yaw_deg is only valid when yaw_mode is fixed")
        return self
