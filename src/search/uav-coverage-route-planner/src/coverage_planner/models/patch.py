"""Search patch models and coverage results."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite

from coverage_planner.models.search_area import Polygonal


@dataclass(frozen=True, slots=True)
class PatchGridConfig:
    """Optional explicit patch dimensions and deterministic grid origin."""

    width_m: float | None = None
    height_m: float | None = None
    origin_enu_m: tuple[float, float] = (0.0, 0.0)
    minimum_clipped_area_m2: float = 1e-9

    def __post_init__(self) -> None:
        for name, value in (("width_m", self.width_m), ("height_m", self.height_m)):
            if value is not None and (not isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be a finite number greater than zero")
        if len(self.origin_enu_m) != 2 or not all(isfinite(value) for value in self.origin_enu_m):
            raise ValueError("origin_enu_m must contain two finite numbers")
        if (not isfinite(self.minimum_clipped_area_m2)
                or self.minimum_clipped_area_m2 < 0.0):
            raise ValueError("minimum_clipped_area_m2 cannot be negative")


@dataclass(frozen=True, slots=True)
class Patch:
    """A clipped cell of searchable ground in ENU coordinates."""

    id: str
    row: int
    column: int
    center_enu_m: tuple[float, float]
    geometry: Polygonal
    area_m2: float
    searchable: bool = True
    covered: bool = False
    coverage_ratio: float = 0.0
    covered_by_waypoint_ids: tuple[str, ...] = ()
    semantic_labels: tuple[str, ...] = ()

    def with_coverage(
        self,
        *,
        covered: bool,
        coverage_ratio: float,
        covered_by_waypoint_ids: tuple[str, ...],
    ) -> Patch:
        return replace(
            self,
            covered=covered,
            coverage_ratio=coverage_ratio,
            covered_by_waypoint_ids=covered_by_waypoint_ids,
        )
