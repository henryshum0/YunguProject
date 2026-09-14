"""Ground-truth search targets shared by the Gazebo spawner and the mock detector.

Both halves of the simulated detection loop read the same declaration, so the
objects the camera sees and the objects the mock projects cannot drift apart.

Positions are ENU metres in the navigation frame, anchored at the drone launch
point, and refer to the target's ground contact point. The spawner converts them
to Gazebo world coordinates with ``world_origin_in_gz``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import isfinite

import numpy as np

from detection.camera import box_corners


class TargetConfigError(ValueError):
    """Raised when a ground-truth target declaration is unusable."""


@dataclass(frozen=True, slots=True)
class GroundTruthTarget:
    """One simulated target: what it is, where it stands, and how big it is."""

    target_id: str
    class_id: str
    position: tuple[float, float, float]
    yaw_deg: float
    size: tuple[float, float, float]
    model: str

    def __post_init__(self) -> None:
        if not self.target_id:
            raise TargetConfigError("target id must not be empty")
        if not self.class_id:
            raise TargetConfigError(f"target '{self.target_id}' must declare a class")
        if not all(isfinite(value) for value in (*self.position, self.yaw_deg)):
            raise TargetConfigError(f"target '{self.target_id}' position and yaw must be finite")
        if len(self.size) != 3 or not all(isfinite(v) and v > 0.0 for v in self.size):
            raise TargetConfigError(
                f"target '{self.target_id}' size must be three positive numbers")

    @property
    def center(self) -> tuple[float, float, float]:
        """Centre of the target's bounding box (its position is the ground contact)."""
        return (self.position[0], self.position[1], self.position[2] + self.size[2] / 2.0)

    def corners(self) -> np.ndarray:
        """The eight ENU corners of the target's oriented bounding box."""
        return box_corners(self.center, self.size, self.yaw_deg)

    def gazebo_position(self, world_origin_in_gz: Sequence[float]) -> tuple[float, float, float]:
        """This target's ground contact point in Gazebo world coordinates."""
        offset = tuple(float(value) for value in world_origin_in_gz)
        if len(offset) != 3 or not all(isfinite(value) for value in offset):
            raise TargetConfigError("world_origin_in_gz must be three finite numbers")
        return (self.position[0] + offset[0],
                self.position[1] + offset[1],
                self.position[2] + offset[2])


@dataclass(frozen=True, slots=True)
class TargetField:
    """Every ground-truth target plus what the spawner needs to place them."""

    targets: tuple[GroundTruthTarget, ...]
    world_origin_in_gz: tuple[float, float, float]

    def __post_init__(self) -> None:
        identifiers = [target.target_id for target in self.targets]
        duplicates = {name for name in identifiers if identifiers.count(name) > 1}
        if duplicates:
            raise TargetConfigError(f"duplicate target ids: {', '.join(sorted(duplicates))}")

    def of_classes(self, classes: Sequence[str]) -> tuple[GroundTruthTarget, ...]:
        """Targets whose true class is one of ``classes``."""
        wanted = {str(name) for name in classes}
        return tuple(target for target in self.targets if target.class_id in wanted)


def parse_targets(payload: Mapping, *, ground_z_m: float) -> TargetField:
    """Build a :class:`TargetField` from the parsed ``targets.yaml`` mapping.

    Targets whose ``position`` gives only ``[x, y]`` stand on ``ground_z_m``.
    """
    if not isinstance(payload, Mapping):
        raise TargetConfigError("targets configuration must be a mapping")
    origin = payload.get("world_origin_in_gz", (0.0, 0.0, 0.0))
    if not isinstance(origin, Sequence) or len(origin) != 3:
        raise TargetConfigError("world_origin_in_gz must be three numbers")
    models = payload.get("models", {})
    if not isinstance(models, Mapping):
        raise TargetConfigError("models must be a mapping of class name to Gazebo model")
    entries = payload.get("targets", [])
    if not isinstance(entries, Sequence) or isinstance(entries, (str, bytes)):
        raise TargetConfigError("targets must be a sequence of target mappings")

    parsed = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            raise TargetConfigError(f"target #{index} must be a mapping")
        target_id = str(entry.get("id", "")).strip()
        class_id = str(entry.get("class", "")).strip()
        position = entry.get("position")
        if not isinstance(position, Sequence) or len(position) not in (2, 3):
            raise TargetConfigError(
                f"target '{target_id or index}' position must be [x, y] or [x, y, z]")
        z = float(position[2]) if len(position) == 3 else float(ground_z_m)
        size = entry.get("size")
        if not isinstance(size, Sequence) or len(size) != 3:
            raise TargetConfigError(
                f"target '{target_id or index}' size must be [length, width, height]")
        model = str(entry.get("model") or models.get(class_id, "")).strip()
        if not model:
            raise TargetConfigError(
                f"target '{target_id or index}' has no Gazebo model: add one under models.{class_id}")
        parsed.append(GroundTruthTarget(
            target_id=target_id,
            class_id=class_id,
            position=(float(position[0]), float(position[1]), z),
            yaw_deg=float(entry.get("yaw_deg", 0.0)),
            size=(float(size[0]), float(size[1]), float(size[2])),
            model=model,
        ))
    return TargetField(
        targets=tuple(parsed),
        world_origin_in_gz=(float(origin[0]), float(origin[1]), float(origin[2])),
    )
