"""Altitude-dependent building obstacle selection."""

from __future__ import annotations

from dataclasses import dataclass

from shapely import unary_union
from shapely.geometry import MultiPolygon, Polygon

from coverage_planner.io.semantic_map import building_safety_elevations, building_safety_geometry
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.semantic_map import SemanticMap


@dataclass(frozen=True, slots=True)
class FlightObstacles:
    building_ids: tuple[str, ...]
    geometry: Polygonal
    horizontal_clearance_m: float


def select_flight_obstacles(
    semantic_map: SemanticMap, *, flight_altitude_m: float,
    vertical_clearance_m: float, horizontal_clearance_m: float,
    allow_overflight_above_buildings: bool = True,
) -> FlightObstacles:
    if vertical_clearance_m < 0 or horizontal_clearance_m < 0:
        raise ValueError("clearance values cannot be negative")
    selected = []
    for node in semantic_map.building_nodes:
        elevation_min_m, elevation_max_m = building_safety_elevations(semantic_map, node)
        blocked_by_height = (
            elevation_min_m - vertical_clearance_m
            <= flight_altitude_m
            <= elevation_max_m + vertical_clearance_m
        )
        if not allow_overflight_above_buildings or blocked_by_height:
            selected.append(node)
    buffered = [
        building_safety_geometry(semantic_map, node).buffer(
            horizontal_clearance_m, join_style="mitre")
        for node in selected
    ]
    geometry = unary_union(buffered) if buffered else Polygon()
    if isinstance(geometry, (Polygon, MultiPolygon)):
        polygonal = geometry
    else:
        polygonal = Polygon()
    return FlightObstacles(tuple(node.id for node in selected), polygonal, horizontal_clearance_m)
