"""Geometry conversion for the planner's compact internal map model."""

from __future__ import annotations

from shapely.geometry import Polygon, box

from coverage_planner.models.semantic_map import (
    BuildingShape,
    PolygonShape,
    RectangleShape,
    SemanticMap,
    SemanticNode,
)


class SemanticMapError(ValueError):
    """Raised when internal map geometry is invalid."""


def rectangle_geometry(shape: RectangleShape) -> Polygon:
    return box(*shape.min_corner, *shape.max_corner)


def building_shape_geometry(shape: BuildingShape) -> Polygon:
    if isinstance(shape, RectangleShape):
        return rectangle_geometry(shape)
    if not isinstance(shape, PolygonShape):
        raise TypeError(f"unsupported occupied-area shape: {type(shape).__name__}")
    geometry = Polygon(shape.coords)
    if geometry.is_empty or not geometry.is_valid or geometry.area <= 0:
        raise SemanticMapError("building polygon must be valid and have positive area")
    return geometry


def building_safety_geometry(semantic_map: SemanticMap, node: SemanticNode) -> Polygon:
    override = semantic_map.building_safety_overrides.get(node.id)
    if override is None:
        return building_shape_geometry(node.shape)
    return box(*override.min_corner, *override.max_corner)


def building_safety_elevations(
    semantic_map: SemanticMap, node: SemanticNode,
) -> tuple[float, float]:
    override = semantic_map.building_safety_overrides.get(node.id)
    if override is None:
        return node.properties.elevation_min_m, node.properties.elevation_max_m
    return override.elevation_min_m, override.elevation_max_m


def search_area_geometry(semantic_map: SemanticMap) -> Polygon:
    return Polygon(semantic_map.search_area.coords)


def excluded_search_geometries(semantic_map: SemanticMap) -> tuple[Polygon, ...]:
    """Return map-authored ground regions that require no search."""
    return tuple(building_shape_geometry(region.shape)
                 for region in semantic_map.excluded_search_regions)
