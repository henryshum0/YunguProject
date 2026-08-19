"""Semantic-map loading and geometry conversion."""

from __future__ import annotations

import json
from pathlib import Path

from pydantic import ValidationError
from shapely import box
from shapely.geometry import Polygon

from coverage_planner.models.semantic_map import RectangleShape, SemanticMap, SemanticNode


class SemanticMapError(ValueError):
    """Raised when a semantic-map file cannot be read or validated."""


def load_semantic_map(path: str | Path) -> SemanticMap:
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        return SemanticMap.model_validate(payload)
    except OSError as exc:
        raise SemanticMapError(f"cannot read semantic map {source}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise SemanticMapError(f"invalid JSON in semantic map {source}: {exc}") from exc
    except ValidationError as exc:
        raise SemanticMapError(f"invalid semantic map {source}: {exc}") from exc


def rectangle_geometry(shape: RectangleShape) -> Polygon:
    return box(*shape.min_corner, *shape.max_corner)


def building_safety_geometry(semantic_map: SemanticMap, node: SemanticNode) -> Polygon:
    override = semantic_map.building_safety_overrides.get(node.id)
    if override is None:
        return rectangle_geometry(node.shape)
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
    return tuple(rectangle_geometry(region.shape)
                 for region in semantic_map.excluded_search_regions)
