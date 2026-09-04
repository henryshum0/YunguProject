"""Global Scanline coverage-generation method."""

from __future__ import annotations

from dataclasses import dataclass

from coverage_planner.coverage.scanlines import CapturePlan, generate_capture_plan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal


@dataclass(frozen=True, slots=True)
class GlobalScanlineGenerator:
    """Lay a global scanline lattice, then clip it by free-ground geometry."""

    method: str = "global_scanline"

    def generate(
        self,
        geometry: Polygonal,
        *,
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        scan_direction_deg: float,
    ) -> CapturePlan:
        return generate_capture_plan(
            geometry,
            camera=camera,
            flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            scan_direction_deg=scan_direction_deg,
        )
