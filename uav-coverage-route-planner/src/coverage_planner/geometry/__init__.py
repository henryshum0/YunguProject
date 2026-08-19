"""Geometry and coordinate conversion utilities."""

from coverage_planner.geometry.calibration import CalibrationError, MapCalibration
from coverage_planner.geometry.search_area import SearchAreaError, build_effective_search_area

__all__ = [
    "CalibrationError",
    "MapCalibration",
    "SearchAreaError",
    "build_effective_search_area",
]
