"""Match estimated target positions against the simulated ground truth.

Simulation only, and deliberately outside the detection path. What a search
reports is what it *estimated*: a box back-projected onto the ground plane and
averaged over frames. A real system has nothing else, and neither does the skill
layer, so the estimate must never be quietly corrected with knowledge only the
simulator has.

This module is for reading the result afterwards. It pairs each estimate with
the ground-truth target it most likely refers to, so an operator can see the
error at a glance instead of comparing numbers by hand — and so a class mistake
is visible, because the truth's own class is reported next to it.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import hypot

from detection.targets import GroundTruthTarget, TargetField


#: How far an estimate may sit from a target and still be taken to mean it.
#: Wider than the localization error a single frame can have, narrower than the
#: spacing between distinct targets.
DEFAULT_MATCH_RADIUS_M = 15.0


@dataclass(frozen=True, slots=True)
class TruthMatch:
    """The ground-truth target an estimate refers to, and how far off it was."""

    target_id: str
    class_id: str
    position: tuple[float, float, float]
    #: Horizontal distance between the estimate and the true position, in metres.
    error_m: float


def match_to_truth(
    positions: Sequence[Sequence[float]],
    field: TargetField,
    *,
    max_distance_m: float = DEFAULT_MATCH_RADIUS_M,
) -> tuple[TruthMatch | None, ...]:
    """Pair each estimated position with a ground-truth target.

    Returns one entry per input position, in the same order, or ``None`` where no
    target is close enough — which is what a false positive confirmed by mistake
    looks like. Assignment is greedy by distance and one-to-one, so two estimates
    of the same object cannot both claim the same target.
    """
    candidates = []
    for index, position in enumerate(positions):
        x, y = float(position[0]), float(position[1])
        for target in field.targets:
            distance = hypot(target.position[0] - x, target.position[1] - y)
            if distance <= max_distance_m:
                candidates.append((distance, index, target))
    candidates.sort(key=lambda item: item[0])

    matches: dict[int, TruthMatch] = {}
    claimed: set[str] = set()
    for distance, index, target in candidates:
        if index in matches or target.target_id in claimed:
            continue
        matches[index] = _match(target, distance)
        claimed.add(target.target_id)
    return tuple(matches.get(index) for index in range(len(positions)))


def _match(target: GroundTruthTarget, distance: float) -> TruthMatch:
    return TruthMatch(
        target_id=target.target_id,
        class_id=target.class_id,
        position=target.position,
        error_m=distance,
    )
