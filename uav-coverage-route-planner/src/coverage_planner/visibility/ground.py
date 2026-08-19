"""Ground visibility after conservative building-wall occlusion clipping."""

from __future__ import annotations

from shapely import convex_hull, unary_union
from shapely.geometry import MultiPoint, Polygon
from shapely.geometry.base import BaseGeometry

from coverage_planner.camera import ground_footprint_polygon
from coverage_planner.io.semantic_map import building_safety_elevations, building_safety_geometry
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.semantic_map import SemanticMap


def visible_detection_ground(
    camera: CameraConfig, *, center_enu_m: tuple[float, float], flight_altitude_m: float,
    ground_elevation_m: float, yaw_deg: float, semantic_map: SemanticMap,
) -> Polygonal:
    """Return ground positions where the configured static target is fully visible.

    The camera footprint already accounts for target dimensions, target height,
    and an image-edge margin.  This function subtracts building footprints and
    the conservative ground shadows cast by their vertical walls.
    """
    footprint = ground_footprint_polygon(
        camera, center_enu_m=center_enu_m, flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg)
    shadows: list[BaseGeometry] = []
    camera_x, camera_y = center_enu_m
    target_plane_z = ground_elevation_m + camera.target_height_m
    for node in semantic_map.building_nodes:
        base = building_safety_geometry(semantic_map, node)
        _, height = building_safety_elevations(semantic_map, node)
        if height <= target_plane_z:
            shadows.append(base)
            continue
        projected = []
        denominator = flight_altitude_m - height
        for x, y in list(base.exterior.coords)[:-1]:
            if denominator > 1e-6:
                scale = (flight_altitude_m - target_plane_z) / denominator
            else:
                scale = 1000.0
            projected.append((camera_x + scale * (x - camera_x),
                              camera_y + scale * (y - camera_y)))
        shadow = convex_hull(MultiPoint([*list(base.exterior.coords)[:-1], *projected]))
        shadows.append(shadow)
    occluded = unary_union(shadows) if shadows else Polygon()
    visible = footprint.difference(occluded)
    if isinstance(visible, (Polygon,)):
        return visible
    if visible.geom_type == "MultiPolygon":
        return visible  # type: ignore[return-value]
    return Polygon()
