"""Continuous-flight mission models for a lower-level vehicle adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

SegmentKind = Literal["coverage_lane", "connector", "obstacle_avoidance", "return_home"]


@dataclass(frozen=True, slots=True)
class FlightWaypoint:
    """A route control point; video detection is continuous between these points."""

    id: str
    sequence: int
    x: float
    y: float
    z: float
    heading_deg: float
    speed_mps: float
    turn_in_place: bool = False
    hold_time_s: float = 0.0


@dataclass(frozen=True, slots=True)
class RouteSegment:
    """One constant-command straight segment of the planned flight route."""

    id: str
    sequence: int
    kind: SegmentKind
    start_waypoint_id: str
    end_waypoint_id: str
    heading_deg: float
    speed_mps: float
    length_m: float
    detection_enabled: bool
    source_scan_line_index: int | None = None
    source_scan_segment_index: int | None = None
    source_coverage_cell_index: int | None = None


@dataclass(frozen=True, slots=True)
class CoverageLane:
    """A maximal consecutive run of parallel ground-coverage segments."""

    id: str
    sequence: int
    heading_deg: float
    speed_mps: float
    route_segment_ids: tuple[str, ...]
    length_m: float


@dataclass(frozen=True, slots=True)
class ContinuousFlightPlan:
    """Continuous video-detection route and its explicit flight commands."""

    video_analysis_rate_hz: float
    control_point_spacing_m: float
    lane_overlap: float
    forward_overlap: float
    target_width_m: float
    target_length_m: float
    target_height_m: float
    image_boundary_margin_ratio: float
    waypoints: tuple[FlightWaypoint, ...]
    route_segments: tuple[RouteSegment, ...]
    lanes: tuple[CoverageLane, ...]
    visibility_sample_count: int
