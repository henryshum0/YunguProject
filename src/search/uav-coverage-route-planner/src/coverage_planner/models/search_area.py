"""Models describing the result of effective search-area construction."""

from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry import MultiPolygon, Polygon

Polygonal = Polygon | MultiPolygon


@dataclass(frozen=True, slots=True)
class SearchAreaMetrics:
    """Auditable area measurements in square metres."""

    requested_area_m2: float
    within_map_area_m2: float
    outside_map_area_m2: float
    building_excluded_area_m2: float
    explicit_excluded_area_m2: float
    effective_search_area_m2: float


@dataclass(frozen=True, slots=True)
class EffectiveSearchArea:
    """Searchable outdoor ground and the geometries used to derive it."""

    requested_geometry: Polygonal
    map_geometry: Polygon
    clipped_geometry: Polygonal
    building_exclusion_geometry: Polygonal
    explicit_exclusion_geometry: Polygonal
    geometry: Polygonal
    metrics: SearchAreaMetrics
