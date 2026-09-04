#!/usr/bin/env python3
"""Convert a binary STL collision mesh into coverage_planner map geometry."""

from __future__ import annotations

import argparse
import json
from math import isfinite
from pathlib import Path
import struct
import sys
from typing import Iterable

from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
from shapely.ops import unary_union

Point3 = tuple[float, float, float]
Triangle = tuple[Point3, Point3, Point3]


def read_binary_stl(path: Path) -> tuple[Triangle, ...]:
    """Read a binary STL, rejecting malformed and ASCII files clearly."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"'{path}' is too short to be a binary STL")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(
            f"'{path}' is not a valid binary STL: expected {expected_size} bytes for "
            f"{triangle_count} triangles, found {len(data)}")
    triangles: list[Triangle] = []
    offset = 84
    for index in range(triangle_count):
        values = struct.unpack_from("<12f", data, offset)
        vertices = (
            (values[3], values[4], values[5]),
            (values[6], values[7], values[8]),
            (values[9], values[10], values[11]),
        )
        if not all(isfinite(value) for vertex in vertices for value in vertex):
            raise ValueError(f"triangle {index} in '{path}' has a non-finite coordinate")
        triangles.append(vertices)
        offset += 50
    return tuple(triangles)


def connected_components(triangles: Iterable[Triangle]) -> tuple[tuple[Triangle, ...], ...]:
    """Group triangles that share one or more exact STL vertices."""
    indexed = tuple(triangles)
    owners: dict[Point3, list[int]] = {}
    for triangle_index, triangle in enumerate(indexed):
        for vertex in triangle:
            owners.setdefault(vertex, []).append(triangle_index)
    adjacency = [set() for _ in indexed]
    for triangle_indices in owners.values():
        first = triangle_indices[0]
        for other in triangle_indices[1:]:
            adjacency[first].add(other)
            adjacency[other].add(first)
    seen: set[int] = set()
    components: list[tuple[Triangle, ...]] = []
    for start in range(len(indexed)):
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        component: list[Triangle] = []
        while stack:
            current = stack.pop()
            component.append(indexed[current])
            for neighbor in adjacency[current]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        components.append(tuple(component))
    return tuple(components)


def component_footprints(
    components: Iterable[tuple[Triangle, ...]], minimum_height_m: float,
) -> tuple[Polygon, ...]:
    """Project raised components onto XY and return simple obstacle polygons."""
    footprints: list[Polygon] = []
    for component in components:
        vertices = tuple(vertex for triangle in component for vertex in triangle)
        if max(vertex[2] for vertex in vertices) - min(vertex[2] for vertex in vertices) <= minimum_height_m:
            continue
        triangles = []
        for triangle in component:
            projection = Polygon([(vertex[0], vertex[1]) for vertex in triangle])
            if not projection.is_empty and projection.area > 0.0:
                triangles.append(projection)
        if not triangles:
            raise ValueError("a raised collider component has no usable XY footprint")
        polygons = tuple(_polygons(unary_union(triangles)))
        if not polygons:
            raise ValueError("a raised collider component has no usable XY footprint")
        for polygon in polygons:
            # Obstacles are conservative: interior mesh holes are non-flyable too.
            footprint = Polygon(polygon.exterior.coords)
            if not footprint.is_valid or footprint.area <= 0.0:
                raise ValueError("a collider component produced an invalid XY footprint")
            footprints.append(footprint)
    return tuple(sorted(footprints, key=_polygon_sort_key))


def _polygons(geometry) -> Iterable[Polygon]:
    if isinstance(geometry, Polygon):
        yield geometry
    elif isinstance(geometry, (MultiPolygon, GeometryCollection)):
        for part in geometry.geoms:
            yield from _polygons(part)


def _polygon_sort_key(polygon: Polygon) -> tuple[float, float, float, float, float]:
    min_x, min_y, max_x, max_y = polygon.bounds
    return (min_x, min_y, max_x, max_y, polygon.area)


def map_payload(footprints: Iterable[Polygon], origin_x: float, origin_y: float) -> dict[str, object]:
    """Create the strict map JSON payload consumed by coverage_planner."""
    return {
        "schema_version": "1.0",
        "origin": {"x": _json_number(origin_x), "y": _json_number(origin_y)},
        "occupied_areas": [
            {
                "id": f"collider_{index:03d}",
                "points": [
                    [_json_number(x), _json_number(y)]
                    for x, y in list(polygon.exterior.coords)[:-1]
                ],
            }
            for index, polygon in enumerate(footprints, start=1)
        ],
    }


def _json_number(value: float) -> float:
    rounded = round(float(value), 9)
    return 0.0 if rounded == 0.0 else rounded


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stl_file", type=Path, help="Input binary STL collider mesh")
    parser.add_argument("--output", required=True, type=Path, help="Output planner map JSON")
    parser.add_argument("--origin-x", type=float, default=0.0, help="Map origin x in ENU metres")
    parser.add_argument("--origin-y", type=float, default=0.0, help="Map origin y in ENU metres")
    parser.add_argument(
        "--minimum-height-m", type=float, default=0.1,
        help="Discard components no taller than this value (default: 0.1)",
    )
    args = parser.parse_args(argv)
    if not isfinite(args.origin_x) or not isfinite(args.origin_y):
        parser.error("origin coordinates must be finite")
    if not isfinite(args.minimum_height_m) or args.minimum_height_m < 0.0:
        parser.error("--minimum-height-m must be finite and non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    try:
        footprints = component_footprints(
            connected_components(read_binary_stl(args.stl_file)), args.minimum_height_m)
        payload = map_payload(footprints, args.origin_x, args.origin_y)
        args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, struct.error) as exc:
        print(f"stl_to_planner_map: {exc}", file=sys.stderr)
        return 2
    print(f"wrote {len(payload['occupied_areas'])} occupied areas to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
