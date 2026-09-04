"""Validated domain models."""

from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.config import (
    FlightConfig,
    MapConfig,
    OccupiedArea,
    OutputTopics,
    Origin,
    PlannerConfig,
    StartupConfig,
)
from coverage_planner.models.flight import (
    ContinuousFlightPlan,
    CoverageLane,
    FlightWaypoint,
    RouteSegment,
)
from coverage_planner.models.patch import Patch, PatchGridConfig
from coverage_planner.models.search_area import EffectiveSearchArea, SearchAreaMetrics
from coverage_planner.models.semantic_map import (
    PolygonShape,
    SearchArea,
    SemanticMap,
    SemanticNode,
    SemanticProperties,
)
from coverage_planner.models.waypoint import ScanSegment, Waypoint

__all__ = [
    "CameraConfig",
    "ContinuousFlightPlan",
    "CoverageLane",
    "EffectiveSearchArea",
    "FlightConfig",
    "FlightWaypoint",
    "MapConfig",
    "OccupiedArea",
    "OutputTopics",
    "Origin",
    "Patch",
    "PatchGridConfig",
    "PlannerConfig",
    "PolygonShape",
    "RouteSegment",
    "ScanSegment",
    "SearchArea",
    "SearchAreaMetrics",
    "SemanticMap",
    "SemanticNode",
    "SemanticProperties",
    "StartupConfig",
    "Waypoint",
]
