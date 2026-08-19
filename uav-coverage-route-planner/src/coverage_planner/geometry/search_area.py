"""Effective outdoor search-area construction with Shapely operations."""

from __future__ import annotations

from collections.abc import Iterable

from shapely import unary_union
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.validation import explain_validity

from coverage_planner.io.semantic_map import (
    building_safety_geometry,
    excluded_search_geometries,
    search_area_geometry,
)
from coverage_planner.models.search_area import (
    EffectiveSearchArea,
    Polygonal,
    SearchAreaMetrics,
)
from coverage_planner.models.semantic_map import SemanticMap


class SearchAreaError(ValueError):
    """Raised when an effective search area cannot be constructed."""


def build_effective_search_area(
    semantic_map: SemanticMap,
    user_search_geometry: BaseGeometry,
    *,
    excluded_regions: Iterable[BaseGeometry] = (),
) -> EffectiveSearchArea:
    """Clip a request to map bounds and subtract non-searchable ground."""
    requested = _validate_polygonal(user_search_geometry, "user search geometry")
    map_geometry = search_area_geometry(semantic_map)
    if not map_geometry.is_valid:
        raise SearchAreaError(f"semantic map search area is invalid: {explain_validity(map_geometry)}")

    clipped = _polygonal_parts(requested.intersection(map_geometry))
    building_union = _union_polygonal(
        building_safety_geometry(semantic_map, node)
        for node in semantic_map.building_nodes
        if node.properties.ground_contact
    )
    explicit_union = _union_polygonal(
        _validate_polygonal(region, "excluded region")
        for region in (*excluded_search_geometries(semantic_map), *tuple(excluded_regions))
    )

    buildings_in_request = _polygonal_parts(clipped.intersection(building_union))
    after_buildings = _polygonal_parts(clipped.difference(building_union))
    explicit_in_free_ground = _polygonal_parts(after_buildings.intersection(explicit_union))
    effective = _polygonal_parts(after_buildings.difference(explicit_union))

    metrics = SearchAreaMetrics(
        requested_area_m2=requested.area,
        within_map_area_m2=clipped.area,
        outside_map_area_m2=requested.area - clipped.area,
        building_excluded_area_m2=buildings_in_request.area,
        explicit_excluded_area_m2=explicit_in_free_ground.area,
        effective_search_area_m2=effective.area,
    )
    return EffectiveSearchArea(
        requested_geometry=requested,
        map_geometry=map_geometry,
        clipped_geometry=clipped,
        building_exclusion_geometry=buildings_in_request,
        explicit_exclusion_geometry=explicit_in_free_ground,
        geometry=effective,
        metrics=metrics,
    )


def _validate_polygonal(geometry: BaseGeometry, label: str) -> Polygonal:
    if geometry.is_empty:
        raise SearchAreaError(f"{label} cannot be empty")
    if not isinstance(geometry, (Polygon, MultiPolygon)):
        raise SearchAreaError(f"{label} must be Polygon or MultiPolygon, got {geometry.geom_type}")
    if not geometry.is_valid:
        raise SearchAreaError(f"{label} is invalid: {explain_validity(geometry)}")
    return geometry


def _union_polygonal(geometries: Iterable[Polygonal]) -> Polygonal:
    items = tuple(geometries)
    if not items:
        return Polygon()
    return _polygonal_parts(unary_union(items))


def _polygonal_parts(geometry: BaseGeometry) -> Polygonal:
    if geometry.is_empty:
        return Polygon()
    if isinstance(geometry, (Polygon, MultiPolygon)):
        return geometry
    if isinstance(geometry, GeometryCollection):
        polygons = [part for part in geometry.geoms if isinstance(part, (Polygon, MultiPolygon))]
        return _union_polygonal(polygons)
    raise SearchAreaError(f"geometry operation produced unsupported {geometry.geom_type}")
