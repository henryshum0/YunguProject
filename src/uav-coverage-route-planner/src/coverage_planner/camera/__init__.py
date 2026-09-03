"""Camera projection models."""

from coverage_planner.camera.nadir import (
    CameraGeometryError,
    GroundFootprintDimensions,
    ground_footprint_dimensions,
    ground_footprint_polygon,
)

__all__ = [
    "CameraGeometryError",
    "GroundFootprintDimensions",
    "ground_footprint_dimensions",
    "ground_footprint_polygon",
]
