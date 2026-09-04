"""Protocol shared by coverage geometry generators."""

from __future__ import annotations

from typing import Protocol

from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal


class CoverageStructureGenerator(Protocol):
    """Generate geometric coverage tasks before combinatorial routing."""

    @property
    def method(self) -> str: ...

    def generate(
        self,
        geometry: Polygonal,
        *,
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        scan_direction_deg: float,
    ) -> CapturePlan: ...
