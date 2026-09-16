"""Turn image-plane detections into world-frame ground positions.

The detector answers "what is in this frame"; it cannot answer "where is that on
the map". That second step is this module, and it is deliberately separate: it
runs unchanged whether the boxes came from the mock detector or from the real
RemDet model, and it is what lets a search skill act on a detection.

A box is back-projected by casting the ray through its ground-contact point
(the bottom edge centre of an upright target) and intersecting it with the
ground plane. That needs the vehicle pose at the *capture* time of the frame,
not at the time the detection arrives, which is why poses are buffered and
looked up by timestamp.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from math import isfinite

import numpy as np

from detection.camera import Extrinsics, PinholeCamera, PixelBox


@dataclass(frozen=True, slots=True)
class TimedPose:
    """A vehicle pose with the time it was valid at."""

    stamp_sec: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def body_in_world(self) -> Extrinsics:
        return Extrinsics.from_quaternion(self.position, self.orientation)


class PoseBuffer:
    """Recent vehicle poses, looked up by the timestamp of a detection frame."""

    def __init__(self, *, history_sec: float) -> None:
        if not isfinite(history_sec) or history_sec <= 0.0:
            raise ValueError("history_sec must be a positive finite number")
        self._history_sec = history_sec
        self._poses: deque[TimedPose] = deque()

    def __len__(self) -> int:
        return len(self._poses)

    def add(self, pose: TimedPose) -> None:
        """Append a pose and drop everything older than the history window."""
        self._poses.append(pose)
        horizon = pose.stamp_sec - self._history_sec
        while self._poses and self._poses[0].stamp_sec < horizon:
            self._poses.popleft()

    def nearest(self, stamp_sec: float, *, max_age_sec: float) -> TimedPose | None:
        """Return the pose closest in time to ``stamp_sec``, or ``None`` if too stale.

        Nearest rather than interpolated: the pose stream runs far faster than
        the detector, so the closest sample is already within a few milliseconds,
        and interpolating quaternions would add error-prone machinery for no
        measurable gain.
        """
        if not self._poses:
            return None
        stamps = [pose.stamp_sec for pose in self._poses]
        index = bisect_left(stamps, stamp_sec)
        candidates = []
        if index < len(stamps):
            candidates.append(self._poses[index])
        if index > 0:
            candidates.append(self._poses[index - 1])
        best = min(candidates, key=lambda pose: abs(pose.stamp_sec - stamp_sec))
        if abs(best.stamp_sec - stamp_sec) > max_age_sec:
            return None
        return best


def anchor_pixel(box: PixelBox, anchor: str) -> tuple[float, float]:
    """Pixel of the box that is assumed to touch the ground."""
    if anchor == "bottom_center":
        return (box.center_x, box.max_y)
    if anchor == "center":
        return (box.center_x, box.center_y)
    raise ValueError(f"unsupported localizer anchor '{anchor}'")


def ground_intersection(
    camera: PinholeCamera,
    camera_extrinsics: Extrinsics,
    pose: TimedPose,
    pixel: tuple[float, float],
    *,
    ground_z_m: float,
    max_range_m: float,
) -> tuple[tuple[float, float, float], float] | None:
    """Back-project a pixel onto the ground plane.

    Returns the ENU ground point and its range from the camera, or ``None`` when
    the ray points at or above the horizon, lands behind the camera, or lands
    beyond ``max_range_m``.
    """
    camera_in_world = pose.body_in_world().compose(camera_extrinsics)
    origin = camera_in_world.translation
    direction = camera_in_world.direction_to_parent(camera.ray_direction(*pixel))
    if direction[2] >= -1e-6:
        return None
    distance = (ground_z_m - origin[2]) / direction[2]
    if distance <= 0.0 or distance > max_range_m:
        return None
    point = origin + distance * direction
    return ((float(point[0]), float(point[1]), float(point[2])), float(distance))


def ground_position(
    camera: PinholeCamera,
    camera_extrinsics: Extrinsics,
    pose: TimedPose,
    box: PixelBox,
    *,
    anchor: str,
    ground_z_m: float,
    max_range_m: float,
) -> tuple[tuple[float, float, float], float] | None:
    """Back-project a detection box onto the ground plane through its anchor pixel."""
    return ground_intersection(
        camera, camera_extrinsics, pose, anchor_pixel(box, anchor),
        ground_z_m=ground_z_m, max_range_m=max_range_m)


def ground_footprint(
    camera: PinholeCamera,
    camera_extrinsics: Extrinsics,
    pose: TimedPose,
    box: PixelBox,
    *,
    ground_z_m: float,
    max_range_m: float,
) -> tuple[float, float] | None:
    """Estimate the ground extent of a box, for the size of a 3D detection.

    Back-projects the bottom-left and bottom-right corners; the distance between
    them is the target's apparent width on the ground. The depth extent is not
    observable from a single box, so the width is reused for it.
    """
    left = ground_intersection(
        camera, camera_extrinsics, pose, (box.min_x, box.max_y),
        ground_z_m=ground_z_m, max_range_m=max_range_m)
    right = ground_intersection(
        camera, camera_extrinsics, pose, (box.max_x, box.max_y),
        ground_z_m=ground_z_m, max_range_m=max_range_m)
    if left is None or right is None:
        return None
    width = float(np.linalg.norm(np.asarray(left[0]) - np.asarray(right[0])))
    return (width, width)
