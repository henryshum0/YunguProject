"""Search patch models and coverage results."""

from __future__ import annotations

from dataclasses import dataclass, replace

from pydantic import BaseModel, ConfigDict, Field

from coverage_planner.models.search_area import Polygonal


class PatchGridConfig(BaseModel):
    """Optional explicit patch dimensions and deterministic grid origin."""

    model_config = ConfigDict(extra="forbid")

    width_m: float | None = Field(default=None, gt=0.0)
    height_m: float | None = Field(default=None, gt=0.0)
    origin_enu_m: tuple[float, float] = (0.0, 0.0)
    minimum_clipped_area_m2: float = Field(default=1e-9, ge=0.0)


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
