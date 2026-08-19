"""Models for the observed Yungu semantic-map schema."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

Point2D = tuple[float, float]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class SearchArea(StrictModel):
    kind: Literal["rectangle"]
    coords: list[Point2D] = Field(min_length=4)

    @model_validator(mode="after")
    def validate_rectangle(self) -> SearchArea:
        if len(self.coords) != 4:
            raise ValueError("rectangle search_area must have exactly four coordinates")
        if len(set(self.coords)) != 4:
            raise ValueError("search_area coordinates must describe four distinct corners")
        return self


class RectangleShape(StrictModel):
    type: Literal["rectangle"]
    min_corner: Point2D
    max_corner: Point2D

    @model_validator(mode="after")
    def validate_bounds(self) -> RectangleShape:
        if not all(low < high for low, high in zip(self.min_corner, self.max_corner, strict=True)):
            raise ValueError("rectangle min_corner must be lower than max_corner")
        return self


class SemanticProperties(StrictModel):
    category: str = Field(min_length=1)
    type: str = Field(min_length=1)
    label: str = Field(min_length=1)
    passability: str = Field(min_length=1)
    visibility: str = Field(min_length=1)
    elevation_min_m: float
    elevation_max_m: float
    ground_contact: bool = True

    @model_validator(mode="after")
    def validate_elevation(self) -> SemanticProperties:
        if self.elevation_min_m > self.elevation_max_m:
            raise ValueError("elevation_min_m cannot exceed elevation_max_m")
        return self


class SemanticNode(StrictModel):
    id: str = Field(min_length=1)
    properties: SemanticProperties
    shape: RectangleShape


class SafetyVolume(StrictModel):
    min_corner: Point2D
    max_corner: Point2D
    elevation_min_m: float
    elevation_max_m: float

    @model_validator(mode="after")
    def validate_bounds(self) -> SafetyVolume:
        if not all(low < high for low, high in zip(
                self.min_corner, self.max_corner, strict=True)):
            raise ValueError("safety volume min_corner must be lower than max_corner")
        if self.elevation_min_m > self.elevation_max_m:
            raise ValueError("safety volume minimum elevation cannot exceed maximum")
        return self


class ExcludedSearchRegion(StrictModel):
    """Ground that is intentionally outside the detection responsibility."""

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    reason: str = Field(min_length=1)
    shape: RectangleShape


class SemanticMapMetadata(StrictModel):
    ground_truth_excluded: bool
    source: str = Field(min_length=1)


class SemanticMap(StrictModel):
    schema_version: Literal["1.0"]
    world_name: str = Field(min_length=1)
    coordinate_frame: Literal["ENU"]
    units: Literal["meters"]
    search_area: SearchArea
    nodes: list[SemanticNode]
    building_safety_overrides: dict[str, SafetyVolume] = Field(default_factory=dict)
    excluded_search_regions: list[ExcludedSearchRegion] = Field(default_factory=list)
    metadata: SemanticMapMetadata

    @model_validator(mode="after")
    def validate_unique_ids(self) -> SemanticMap:
        ids = [node.id for node in self.nodes]
        if len(ids) != len(set(ids)):
            raise ValueError("semantic node IDs must be unique")
        unknown = set(self.building_safety_overrides).difference(ids)
        if unknown:
            raise ValueError(f"safety overrides reference unknown nodes: {sorted(unknown)}")
        region_ids = [region.id for region in self.excluded_search_regions]
        if len(region_ids) != len(set(region_ids)):
            raise ValueError("excluded search region IDs must be unique")
        return self

    @property
    def building_nodes(self) -> tuple[SemanticNode, ...]:
        return tuple(node for node in self.nodes if node.properties.category == "building")
