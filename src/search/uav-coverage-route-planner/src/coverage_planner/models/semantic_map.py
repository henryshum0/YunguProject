"""Small immutable map model consumed by the coverage core."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from math import isfinite
from types import MappingProxyType

Point2D = tuple[float, float]


@dataclass(frozen=True, slots=True)
class SearchArea:
    coords: tuple[Point2D, ...]


@dataclass(frozen=True, slots=True)
class RectangleShape:
    min_corner: Point2D
    max_corner: Point2D


@dataclass(frozen=True, slots=True)
class PolygonShape:
    coords: tuple[Point2D, ...]


BuildingShape = RectangleShape | PolygonShape


@dataclass(frozen=True, slots=True)
class SemanticProperties:
    category: str = "building"
    elevation_min_m: float = 0.0
    elevation_max_m: float = 0.0
    ground_contact: bool = True

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (
                self.elevation_min_m, self.elevation_max_m)):
            raise ValueError("building elevations must be finite")
        if self.elevation_min_m > self.elevation_max_m:
            raise ValueError("elevation_min_m cannot exceed elevation_max_m")


@dataclass(frozen=True, slots=True)
class SemanticNode:
    id: str
    properties: SemanticProperties
    shape: BuildingShape

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("occupied-area ID cannot be empty")


@dataclass(frozen=True, slots=True)
class SafetyVolume:
    min_corner: Point2D
    max_corner: Point2D
    elevation_min_m: float
    elevation_max_m: float


@dataclass(frozen=True, slots=True)
class ExcludedSearchRegion:
    id: str
    shape: BuildingShape


@dataclass(frozen=True, slots=True)
class SemanticMap:
    """The bounded search map and non-searchable occupied ground."""

    search_area: SearchArea
    nodes: tuple[SemanticNode, ...] = ()
    building_safety_overrides: Mapping[str, SafetyVolume] = field(default_factory=dict)
    excluded_search_regions: tuple[ExcludedSearchRegion, ...] = ()

    def __post_init__(self) -> None:
        ids = tuple(node.id for node in self.nodes)
        if len(ids) != len(set(ids)):
            raise ValueError("occupied-area IDs must be unique")
        unknown = set(self.building_safety_overrides).difference(ids)
        if unknown:
            raise ValueError(f"safety overrides reference unknown nodes: {sorted(unknown)}")
        object.__setattr__(
            self, "building_safety_overrides",
            MappingProxyType(dict(self.building_safety_overrides)),
        )

    @property
    def building_nodes(self) -> tuple[SemanticNode, ...]:
        return tuple(node for node in self.nodes if node.properties.category == "building")
