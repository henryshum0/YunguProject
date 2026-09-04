"""Frozen models for the ROS node's startup configuration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from shapely.geometry import Polygon

from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.semantic_map import (
    PolygonShape,
    SearchArea,
    SemanticMap,
    SemanticNode,
    SemanticProperties,
)

CoverageGenerationMethod = Literal["global_scanline", "bcd"]
RouteOptimizationMethod = Literal["greedy", "two_opt", "or_opt", "heuristic", "exact", "auto"]
CompletionStrategy = Literal["full_greedy", "local_insertion"]


@dataclass(frozen=True, slots=True)
class Origin:
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class OccupiedArea:
    id: str
    points: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class MapConfig:
    """Reusable map geometry loaded by a planner startup configuration."""

    schema_version: str
    origin: Origin
    occupied_areas: tuple[OccupiedArea, ...]


@dataclass(frozen=True, slots=True)
class FlightConfig:
    altitude_m: float
    ground_elevation_m: float = 0.0
    horizontal_clearance_m: float = 3.0


@dataclass(frozen=True, slots=True)
class PlannerConfig:
    coverage_generation_method: CoverageGenerationMethod = "global_scanline"
    scan_direction_deg: float | None = None
    route_optimization_method: RouteOptimizationMethod = "auto"
    completion_strategy: CompletionStrategy = "local_insertion"
    minimum_coverage_ratio: float = 0.99
    return_to_origin: bool = True


@dataclass(frozen=True, slots=True)
class OutputTopics:
    """Absolute ROS topic names for the node's latched startup result."""

    waypoints: str = "/coverage_planner/waypoints"
    markers: str = "/coverage_planner/markers"


@dataclass(frozen=True, slots=True)
class StartupConfig:
    schema_version: str
    map_file: str
    frame_id: str
    origin: Origin
    search_area_points: tuple[tuple[float, float], ...]
    occupied_areas: tuple[OccupiedArea, ...]
    flight: FlightConfig
    camera: CameraConfig
    planner: PlannerConfig
    output_topics: OutputTopics

    @property
    def search_geometry(self) -> Polygon:
        return Polygon(self.search_area_points)

    def to_semantic_map(self) -> SemanticMap:
        properties = SemanticProperties(
            elevation_min_m=self.flight.ground_elevation_m,
            elevation_max_m=self.flight.ground_elevation_m,
            ground_contact=True,
        )
        nodes = tuple(
            SemanticNode(area.id, properties, PolygonShape(area.points))
            for area in self.occupied_areas
        )
        return SemanticMap(SearchArea(self.search_area_points), nodes)
