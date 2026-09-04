"""Planner waypoint models independent of any flight-controller adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from shapely.geometry import Polygon


@dataclass(frozen=True, slots=True)
class Waypoint:
    """An explicit ENU waypoint with optional camera capture."""

    id: str
    sequence: int
    kind: Literal["capture", "transit"]
    x: float
    y: float
    z: float
    yaw_deg: float
    camera_pitch_deg: float
    capture: bool
    scan_line_index: int | None = None
    scan_segment_index: int | None = None
    camera_footprint_enu: Polygon | None = None
    coverage_cell_index: int | None = None
    is_completion: bool = False


@dataclass(frozen=True, slots=True)
class ScanSegment:
    """One collision-unchecked coverage segment inside effective ground geometry."""

    scan_line_index: int
    segment_index: int
    start_enu_m: tuple[float, float]
    end_enu_m: tuple[float, float]
    direction_yaw_deg: float
    capture_waypoint_ids: tuple[str, ...]
    coverage_cell_index: int | None = None
