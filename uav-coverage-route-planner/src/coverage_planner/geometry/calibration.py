"""Pixel-to-ENU map calibration with explicit image-axis handling."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Annotated, Literal, cast

import numpy as np
import numpy.typing as npt
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, model_validator

Point2D = tuple[float, float]
Matrix = npt.NDArray[np.float64]


class CalibrationError(ValueError):
    """Raised when a map calibration cannot be read or validated."""


class CalibrationModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    content_bounds_px: tuple[float, float, float, float] | None = None

    @model_validator(mode="after")
    def validate_content_bounds(self) -> CalibrationModel:
        if self.content_bounds_px is not None:
            min_x, min_y, max_x, max_y = self.content_bounds_px
            if min_x >= max_x or min_y >= max_y:
                raise ValueError("content_bounds_px must have positive width and height")
        return self


class OriginScaleCalibration(CalibrationModel):
    mode: Literal["origin_scale"]
    pixel_origin: Point2D
    enu_origin_m: Point2D = (0.0, 0.0)
    east_direction_image: Point2D = (1.0, 0.0)
    north_direction_image: Point2D = (0.0, -1.0)
    meters_per_pixel: float = Field(gt=0.0)

    @model_validator(mode="after")
    def validate_axes(self) -> OriginScaleCalibration:
        east = np.asarray(self.east_direction_image)
        north = np.asarray(self.north_direction_image)
        if np.linalg.norm(east) == 0 or np.linalg.norm(north) == 0:
            raise ValueError("image direction vectors must be non-zero")
        if abs(float(np.dot(east, north))) > 1e-6 * np.linalg.norm(east) * np.linalg.norm(north):
            raise ValueError("East and North image directions must be perpendicular")
        return self


class BoundsCalibration(CalibrationModel):
    mode: Literal["bounds"]
    image_width_px: int = Field(gt=1)
    image_height_px: int = Field(gt=1)
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    north_is_up: bool = True
    flip_x: bool = False
    flip_y: bool = False

    @model_validator(mode="after")
    def validate_bounds(self) -> BoundsCalibration:
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError("calibration bounds must have positive extent")
        return self


class ControlPoint(CalibrationModel):
    pixel: Point2D
    enu_m: Point2D


class AffineCalibration(CalibrationModel):
    mode: Literal["affine"]
    control_points: list[ControlPoint] = Field(min_length=3)
    maximum_residual_m: float | None = Field(default=None, ge=0.0)


CalibrationConfig = Annotated[
    OriginScaleCalibration | BoundsCalibration | AffineCalibration,
    Field(discriminator="mode"),
]
_CONFIG_ADAPTER: TypeAdapter[CalibrationConfig] = TypeAdapter(CalibrationConfig)


class MapCalibration:
    """Invertible affine mapping between image pixels and local ENU metres."""

    def __init__(self, config: CalibrationConfig) -> None:
        self.config = config
        self._pixel_to_enu = _matrix_from_config(config)
        linear = self._pixel_to_enu[:, :2]
        if abs(float(np.linalg.det(linear))) < 1e-12:
            raise ValueError("calibration transform is singular")
        self._enu_to_pixel = np.linalg.inv(_homogeneous(self._pixel_to_enu))[:2]
        self.residuals_m = _residuals(config, self._pixel_to_enu)
        if (
            isinstance(config, AffineCalibration)
            and config.maximum_residual_m is not None
            and self.maximum_residual_m > config.maximum_residual_m
        ):
            raise ValueError(
                f"calibration residual {self.maximum_residual_m:.3f} m exceeds "
                f"limit {config.maximum_residual_m:.3f} m"
            )

    @classmethod
    def load(cls, path: str | Path) -> MapCalibration:
        source = Path(path)
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
            return cls(_CONFIG_ADAPTER.validate_python(payload))
        except OSError as exc:
            raise CalibrationError(f"cannot read map calibration {source}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise CalibrationError(f"invalid JSON in map calibration {source}: {exc}") from exc
        except (ValidationError, ValueError) as exc:
            raise CalibrationError(f"invalid map calibration {source}: {exc}") from exc

    @property
    def pixel_to_enu_matrix(self) -> Matrix:
        """Return a copy of the 2 x 3 affine transform matrix."""
        return self._pixel_to_enu.copy()

    @property
    def maximum_residual_m(self) -> float:
        return max(self.residuals_m, default=0.0)

    @property
    def rms_residual_m(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.residuals_m)))) if self.residuals_m else 0.0

    def pixel_to_enu(self, pixel: Point2D) -> Point2D:
        result = self._pixel_to_enu @ np.array([*pixel, 1.0])
        return float(result[0]), float(result[1])

    def enu_to_pixel(self, enu_m: Point2D) -> Point2D:
        result = self._enu_to_pixel @ np.array([*enu_m, 1.0])
        return float(result[0]), float(result[1])

    def content_bounds_enu(self, image_size_px: tuple[int, int]) -> tuple[float, float, float, float]:
        """Return the colored map-content bounds, or the full image when unspecified."""
        width, height = image_size_px
        self.validate_image_size(width, height)
        bounds = self.config.content_bounds_px or (0.0, 0.0, width - 1.0, height - 1.0)
        min_px_x, min_px_y, max_px_x, max_px_y = bounds
        corners = tuple(self.pixel_to_enu(point) for point in (
            (min_px_x, min_px_y), (max_px_x, min_px_y),
            (min_px_x, max_px_y), (max_px_x, max_px_y)))
        return (
            min(point[0] for point in corners), min(point[1] for point in corners),
            max(point[0] for point in corners), max(point[1] for point in corners),
        )

    def validate_image_size(self, width_px: int, height_px: int) -> None:
        """Check dimensions declared by a bounds calibration against an image."""
        if width_px <= 1 or height_px <= 1:
            raise ValueError("image dimensions must both be greater than one pixel")
        if isinstance(self.config, BoundsCalibration) and (
            width_px != self.config.image_width_px
            or height_px != self.config.image_height_px
        ):
            raise ValueError(
                "background image dimensions "
                f"{width_px}x{height_px} do not match bounds calibration dimensions "
                f"{self.config.image_width_px}x{self.config.image_height_px}"
            )


def _matrix_from_config(config: CalibrationConfig) -> Matrix:
    if isinstance(config, OriginScaleCalibration):
        east = np.asarray(config.east_direction_image, dtype=float)
        north = np.asarray(config.north_direction_image, dtype=float)
        east /= np.linalg.norm(east)
        north /= np.linalg.norm(north)
        image_axes = np.column_stack((east, north))
        linear = config.meters_per_pixel * np.linalg.inv(image_axes)
        offset = np.asarray(config.enu_origin_m) - linear @ np.asarray(config.pixel_origin)
        return np.column_stack((linear, offset))
    if isinstance(config, BoundsCalibration):
        x0, x1 = ((config.max_x, config.min_x) if config.flip_x else (config.min_x, config.max_x))
        y_up = config.north_is_up != config.flip_y
        y0, y1 = ((config.max_y, config.min_y) if y_up else (config.min_y, config.max_y))
        sx = (x1 - x0) / (config.image_width_px - 1)
        sy = (y1 - y0) / (config.image_height_px - 1)
        return np.array([[sx, 0.0, x0], [0.0, sy, y0]], dtype=float)
    design = np.array([[*point.pixel, 1.0] for point in config.control_points], dtype=float)
    targets = np.array([point.enu_m for point in config.control_points], dtype=float)
    coefficients, _, rank, _ = np.linalg.lstsq(design, targets, rcond=None)
    if rank < 3:
        raise ValueError("affine control points must not be collinear")
    return cast(Matrix, coefficients.T)


def _homogeneous(matrix: Matrix) -> Matrix:
    return np.asarray(
        np.vstack((matrix, np.array([0.0, 0.0, 1.0], dtype=np.float64))),
        dtype=np.float64,
    )


def _residuals(config: CalibrationConfig, matrix: Matrix) -> tuple[float, ...]:
    if not isinstance(config, AffineCalibration):
        return ()
    values = []
    for point in config.control_points:
        predicted = matrix @ np.array([*point.pixel, 1.0])
        values.append(float(np.linalg.norm(predicted - np.asarray(point.enu_m))))
    return tuple(values)
