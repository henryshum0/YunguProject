"""Deterministic regular patch generation over effective search geometry."""

from __future__ import annotations

from math import ceil, floor

from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, box
from shapely.geometry.base import BaseGeometry

from coverage_planner.camera import GroundFootprintDimensions
from coverage_planner.models.patch import Patch, PatchGridConfig
from coverage_planner.models.search_area import Polygonal


class PatchGenerationError(ValueError):
    """Raised when patch dimensions or effective geometry cannot form a grid."""


def generate_patches(
    effective_geometry: Polygonal,
    *,
    config: PatchGridConfig | None = None,
    camera_dimensions: GroundFootprintDimensions | None = None,
) -> tuple[Patch, ...]:
    """Intersect stable grid cells with every part of the effective geometry."""
    if effective_geometry.is_empty:
        return ()
    if not effective_geometry.is_valid:
        raise PatchGenerationError("effective search geometry must be valid")
    config = config or PatchGridConfig()
    width_m, height_m = _resolve_patch_dimensions(config, camera_dimensions)
    origin_x, origin_y = config.origin_enu_m
    min_x, min_y, max_x, max_y = effective_geometry.bounds
    first_column = floor((min_x - origin_x) / width_m)
    final_column = ceil((max_x - origin_x) / width_m) - 1
    first_row = floor((min_y - origin_y) / height_m)
    final_row = ceil((max_y - origin_y) / height_m) - 1

    patches = []
    for row in range(first_row, final_row + 1):
        cell_min_y = origin_y + row * height_m
        for column in range(first_column, final_column + 1):
            cell_min_x = origin_x + column * width_m
            cell = box(
                cell_min_x,
                cell_min_y,
                cell_min_x + width_m,
                cell_min_y + height_m,
            )
            clipped = _polygonal_parts(cell.intersection(effective_geometry))
            if clipped.is_empty or clipped.area < config.minimum_clipped_area_m2:
                continue
            patches.append(Patch(
                id=_patch_id(row, column),
                row=row,
                column=column,
                center_enu_m=(cell_min_x + width_m / 2.0, cell_min_y + height_m / 2.0),
                geometry=clipped,
                area_m2=clipped.area,
            ))
    return tuple(patches)


def _resolve_patch_dimensions(
    config: PatchGridConfig,
    camera_dimensions: GroundFootprintDimensions | None,
) -> tuple[float, float]:
    width_m = config.width_m
    height_m = config.height_m
    if width_m is None and camera_dimensions is not None:
        width_m = camera_dimensions.scan_line_spacing_m
    if height_m is None and camera_dimensions is not None:
        height_m = camera_dimensions.length_m
    if width_m is None or height_m is None:
        raise PatchGenerationError(
            "patch width_m and height_m are required unless derived from camera dimensions"
        )
    return width_m, height_m


def _patch_id(row: int, column: int) -> str:
    return f"patch_r{row}_c{column}"


def _polygonal_parts(geometry: BaseGeometry) -> Polygonal:
    if geometry.is_empty:
        return Polygon()
    if isinstance(geometry, (Polygon, MultiPolygon)):
        return geometry
    if isinstance(geometry, GeometryCollection):
        polygons: list[Polygon] = []
        for part in geometry.geoms:
            if isinstance(part, Polygon):
                polygons.append(part)
            elif isinstance(part, MultiPolygon):
                polygons.extend(part.geoms)
        if not polygons:
            return Polygon()
        return polygons[0] if len(polygons) == 1 else MultiPolygon(polygons)
    return Polygon()
