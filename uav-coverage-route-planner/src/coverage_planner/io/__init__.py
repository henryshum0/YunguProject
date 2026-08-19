"""Input and output adapters."""

from coverage_planner.io.geojson import GeoJSONError, load_polygonal_geojson
from coverage_planner.io.semantic_map import load_semantic_map

__all__ = ["GeoJSONError", "load_polygonal_geojson", "load_semantic_map"]
