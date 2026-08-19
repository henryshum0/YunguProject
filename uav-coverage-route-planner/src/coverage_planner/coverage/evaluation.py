"""Patch coverage evaluation using actual camera footprint geometry."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Literal

from shapely import unary_union
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from coverage_planner.models.patch import Patch

CoverageMode = Literal["center", "area_ratio"]


class CoverageEvaluationError(ValueError):
    """Raised when patch coverage settings are invalid."""


def evaluate_patch_coverage(
    patches: Sequence[Patch],
    camera_footprints: Mapping[str, BaseGeometry],
    *,
    mode: CoverageMode = "area_ratio",
    minimum_coverage_ratio: float = 0.95,
) -> tuple[Patch, ...]:
    """Evaluate patches and record every footprint contributing non-zero area."""
    if not 0.0 <= minimum_coverage_ratio <= 1.0:
        raise CoverageEvaluationError("minimum_coverage_ratio must be between 0 and 1")
    if mode not in {"center", "area_ratio"}:
        raise CoverageEvaluationError(f"unsupported coverage mode {mode!r}")
    footprint_union = unary_union(tuple(camera_footprints.values())) if camera_footprints else Polygon()
    evaluated = []
    for patch in patches:
        contributors = tuple(
            waypoint_id
            for waypoint_id, footprint in camera_footprints.items()
            if footprint.intersection(patch.geometry).area > 0.0
        )
        covered_area_m2 = patch.geometry.intersection(footprint_union).area
        area_ratio = covered_area_m2 / patch.area_m2 if patch.area_m2 > 0.0 else 0.0
        if mode == "center":
            center = Point(patch.center_enu_m)
            covered = footprint_union.covers(center)
        elif mode == "area_ratio":
            covered = area_ratio >= minimum_coverage_ratio
        evaluated.append(patch.with_coverage(
            covered=covered,
            coverage_ratio=area_ratio,
            covered_by_waypoint_ids=contributors,
        ))
    return tuple(evaluated)
