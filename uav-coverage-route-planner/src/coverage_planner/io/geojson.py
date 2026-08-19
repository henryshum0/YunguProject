"""GeoJSON loading for user-supplied polygonal search geometry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

from shapely import unary_union
from shapely.geometry import MultiPolygon, Polygon, shape
from shapely.geometry.base import BaseGeometry

from coverage_planner.models.search_area import Polygonal


class GeoJSONError(ValueError):
    """Raised when GeoJSON cannot provide valid polygonal geometry."""


def load_polygonal_geojson(path: str | Path) -> Polygonal:
    source = Path(path)
    try:
        payload: Any = json.loads(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise GeoJSONError(f"cannot read GeoJSON {source}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise GeoJSONError(f"invalid JSON in GeoJSON {source}: {exc}") from exc
    try:
        return polygonal_geometry_from_geojson(payload)
    except (KeyError, TypeError, ValueError) as exc:
        raise GeoJSONError(f"invalid polygonal GeoJSON {source}: {exc}") from exc


def polygonal_geometry_from_geojson(payload: object) -> Polygonal:
    """Parse polygonal geometry from a geometry, Feature, or FeatureCollection."""
    if not isinstance(payload, dict):
        raise GeoJSONError("top-level value must be an object")
    kind = payload.get("type")
    if kind == "Feature":
        geometry = payload.get("geometry")
        if geometry is None:
            raise GeoJSONError("Feature geometry cannot be null")
        return polygonal_geometry_from_geojson(geometry)
    if kind == "FeatureCollection":
        features = payload.get("features")
        if not isinstance(features, list) or not features:
            raise GeoJSONError("FeatureCollection must contain at least one feature")
        geometries = [polygonal_geometry_from_geojson(feature) for feature in features]
        return _require_polygonal(unary_union(geometries))
    if kind not in {"Polygon", "MultiPolygon"}:
        raise GeoJSONError(f"expected Polygon or MultiPolygon, got {kind!r}")
    return _require_polygonal(shape(cast(dict[str, Any], payload)))


def _require_polygonal(geometry: BaseGeometry) -> Polygonal:
    if geometry.is_empty:
        raise GeoJSONError("geometry cannot be empty")
    if not isinstance(geometry, (Polygon, MultiPolygon)):
        raise GeoJSONError(f"geometry operation produced {geometry.geom_type}, not polygonal data")
    if not geometry.is_valid:
        raise GeoJSONError("geometry is invalid")
    return geometry
