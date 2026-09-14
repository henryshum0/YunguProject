"""The simulation mock detector: known targets plus a camera model become detections.

Why a mock instead of running a real detector on Gazebo images: a simulated
scene is far too coarse for a model trained on real imagery, so its output would
say nothing about real accuracy; and replaying real video inside the simulation
breaks the tie between what the camera sees and where the vehicle is. The mock
takes the third road. It renders nothing. It projects the known ground-truth
targets through the camera model using the vehicle's own state at that instant,
so detections are aligned with the flight by construction, and then applies an
error model calibrated from the real detector, so the *statistics* the search
logic faces match the ones it will face on the vehicle.

What is validated in simulation is therefore the integration logic, not
detection accuracy; accuracy is validated separately on real data.

This module is pure geometry and statistics with no ROS dependency, so the whole
pipeline is unit-testable frame by frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

import numpy as np

from detection.camera import Extrinsics, PinholeCamera, PixelBox
from detection.config import VisibilityConfig
from detection.error_model import DetectionErrorModel
from detection.occlusion import OcclusionMesh
from detection.targets import GroundTruthTarget, TargetField


# Corners closer than this to the image plane have no usable projection.
MIN_DEPTH_M = 0.1


@dataclass(frozen=True, slots=True)
class VehiclePose:
    """The vehicle state one detector frame is generated from."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    stamp_sec: float

    def __post_init__(self) -> None:
        values = (*self.position, *self.orientation, self.stamp_sec)
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("vehicle pose must be finite")

    def body_in_world(self) -> Extrinsics:
        """The body frame's pose in the ENU world frame."""
        return Extrinsics.from_quaternion(self.position, self.orientation)


@dataclass(frozen=True, slots=True)
class MockDetection:
    """One detection, in the same terms a real detector reports."""

    detection_id: str
    class_id: str
    score: float
    box: PixelBox
    #: Ground-truth target behind this detection, or ``None`` for a false positive.
    target_id: str | None
    #: Camera-to-target range in metres, ``None`` for a false positive. Diagnostic
    #: only: a real detector cannot report it, so nothing downstream may use it.
    range_m: float | None


@dataclass(frozen=True, slots=True)
class MockFrame:
    """One detector output frame, with the bookkeeping the node needs."""

    #: Capture time: the vehicle-state timestamp the frame was generated from.
    stamp_sec: float
    #: Wall time the frame may be published at, once inference latency has passed.
    publish_at_sec: float
    detections: tuple[MockDetection, ...]
    #: Targets that were geometrically visible, whether or not they were reported.
    #: Diagnostic only, for judging whether a miss was geometric or statistical.
    visible_target_ids: tuple[str, ...]


class MockDetector:
    """Generate detector frames for the known targets from the vehicle state."""

    def __init__(
        self,
        *,
        camera: PinholeCamera,
        camera_extrinsics: Extrinsics,
        targets: TargetField,
        error_model: DetectionErrorModel,
        visibility: VisibilityConfig,
        occlusion_mesh: OcclusionMesh | None = None,
    ) -> None:
        self._camera = camera
        self._camera_extrinsics = camera_extrinsics
        self._targets = targets
        self._error_model = error_model
        self._visibility = visibility
        self._occlusion_mesh = occlusion_mesh

    @property
    def occlusion_enabled(self) -> bool:
        return self._occlusion_mesh is not None

    def detect(self, pose: VehiclePose, *, frame_index: int) -> MockFrame:
        """Produce the detections a detector would report for this vehicle state."""
        body_in_world = pose.body_in_world()
        camera_in_world = body_in_world.compose(self._camera_extrinsics)
        camera_position = camera_in_world.translation

        detections: list[MockDetection] = []
        visible: list[str] = []
        for target in self._targets.targets:
            box = self._visible_box(target, body_in_world, camera_position)
            if box is None:
                continue
            visible.append(target.target_id)
            detection = self._report(target, box, camera_position, frame_index)
            if detection is not None:
                detections.append(detection)

        for index, (class_id, box, score) in enumerate(
                self._error_model.sample_false_positives(self._camera.width, self._camera.height)):
            detections.append(MockDetection(
                detection_id=f"fp_{index}#{frame_index:06d}",
                class_id=class_id,
                score=score,
                box=box,
                target_id=None,
                range_m=None,
            ))

        return MockFrame(
            stamp_sec=pose.stamp_sec,
            publish_at_sec=pose.stamp_sec + self._error_model.sample_latency(),
            detections=tuple(detections),
            visible_target_ids=tuple(visible),
        )

    def _visible_box(
        self,
        target: GroundTruthTarget,
        body_in_world: Extrinsics,
        camera_position: np.ndarray,
    ) -> PixelBox | None:
        """Return the target's image box, or ``None`` when the camera cannot see it.

        This is the geometric half of the mock: field of view, far clip, image
        cropping, resolvability and line of sight. No randomness is involved, so
        a target that fails here is one the real camera genuinely cannot show.
        """
        center = np.asarray(target.center, dtype=float)
        distance = float(np.linalg.norm(center - camera_position))
        if distance > self._camera.max_range_m:
            return None

        corners_camera = self._camera_extrinsics.to_child(body_in_world.to_child(target.corners()))
        if (corners_camera[:, 0] <= MIN_DEPTH_M).any():
            return None

        pixels = self._camera.project(corners_camera)
        if not np.isfinite(pixels).all():
            return None
        raw_box = PixelBox.bounding(pixels)
        if self._camera.image_overlap_fraction(raw_box) < self._visibility.min_visible_fraction:
            return None
        box = self._camera.clip(raw_box)
        if box.short_side < self._visibility.min_pixel_size:
            return None
        if self._occlusion_mesh is not None and self._occlusion_mesh.segment_blocked(
                camera_position, center):
            return None
        return box

    def _report(
        self,
        target: GroundTruthTarget,
        box: PixelBox,
        camera_position: np.ndarray,
        frame_index: int,
    ) -> MockDetection | None:
        """Apply the error model to a geometrically visible target."""
        if not self._error_model.is_detected(box.short_side):
            return None
        noisy_box = self._camera.clip(self._error_model.jitter_box(box))
        score = self._error_model.sample_score(box.short_side)
        if score < self._error_model.config.score.publish_threshold:
            return None
        distance = float(np.linalg.norm(np.asarray(target.center, dtype=float) - camera_position))
        return MockDetection(
            detection_id=f"{target.target_id}#{frame_index:06d}",
            class_id=self._error_model.reported_class(target.class_id),
            score=score,
            box=noisy_box,
            target_id=target.target_id,
            range_m=distance,
        )
