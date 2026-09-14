"""The mock detector: geometric visibility first, then the error model."""

from __future__ import annotations

from math import cos, radians, sin
from pathlib import Path

import numpy as np
import pytest

from detection.camera import Extrinsics, PinholeCamera
from detection.config import OcclusionConfig, VisibilityConfig
from detection.error_model import (
    DetectionErrorModel,
    ErrorModelConfig,
    FalsePositiveModel,
    LatencyModel,
    LocalizationNoiseModel,
    MissModel,
    ScoreModel,
)
from detection.mock import MockDetector, VehiclePose
from detection.occlusion import OcclusionMesh
from detection.targets import GroundTruthTarget, TargetField


CAMERA = PinholeCamera.from_horizontal_fov(
    width=640, height=480, horizontal_fov_deg=100.0, max_range_m=50.0,
    frame_id="front_camera_link")
EXTRINSICS = Extrinsics.from_rpy((0.30, 0.0, -0.13), 0.0, 60.0, 0.0)
GROUND_Z = -1.654


def visibility(*, min_pixel_size=4.0, min_visible_fraction=0.6) -> VisibilityConfig:
    return VisibilityConfig(
        min_pixel_size=min_pixel_size,
        min_visible_fraction=min_visible_fraction,
        occlusion=OcclusionConfig(enabled=False, collision_mesh="", world_origin_in_gz=(0, 0, 0)),
    )


def perfect_error_model(**overrides) -> DetectionErrorModel:
    """An error model that reports every visible target exactly, for geometry tests."""
    defaults = dict(
        miss=MissModel(half_probability_px=0.001, slope_px=0.001, max_probability=1.0),
        false_positives=FalsePositiveModel(per_frame_rate=0.0),
        localization=LocalizationNoiseModel(0.0, 0.0, 0.0, 0.0),
        score=ScoreModel(min_score=0.9, max_score=0.9, sigma=0.0, publish_threshold=0.0),
        confusion={},
        latency=LatencyModel(mean_sec=0.0, jitter_sec=0.0),
    )
    defaults.update(overrides)
    return DetectionErrorModel(ErrorModelConfig(**defaults), seed=0)


def car(x: float, y: float, target_id: str = "car_01") -> GroundTruthTarget:
    return GroundTruthTarget(target_id=target_id, class_id="car", position=(x, y, GROUND_Z),
                             yaw_deg=0.0, size=(4.4, 1.8, 1.5), model="yungu_car")


def detector(targets, *, error_model=None, mesh=None, **visibility_overrides) -> MockDetector:
    return MockDetector(
        camera=CAMERA, camera_extrinsics=EXTRINSICS,
        targets=TargetField(targets=tuple(targets), world_origin_in_gz=(0.0, 0.0, 1.15392)),
        error_model=error_model or perfect_error_model(),
        visibility=visibility(**visibility_overrides), occlusion_mesh=mesh)


def pose(x=0.0, y=0.0, z=15.0, yaw_deg=0.0, stamp=100.0) -> VehiclePose:
    half = radians(yaw_deg) / 2.0
    return VehiclePose(position=(x, y, z), orientation=(cos(half), 0.0, 0.0, sin(half)),
                       stamp_sec=stamp)


def test_a_target_ahead_and_below_is_seen() -> None:
    frame = detector([car(12.0, 0.0)]).detect(pose(), frame_index=1)
    assert frame.visible_target_ids == ("car_01",)
    assert len(frame.detections) == 1
    detection = frame.detections[0]
    assert detection.target_id == "car_01"
    assert detection.class_id == "car"
    assert detection.detection_id == "car_01#000001"
    assert 0.0 <= detection.box.center_x <= 640.0
    assert detection.range_m is not None and 10.0 < detection.range_m < 25.0


def test_a_target_behind_the_camera_is_not_seen() -> None:
    frame = detector([car(-12.0, 0.0)]).detect(pose(), frame_index=1)
    assert frame.visible_target_ids == ()
    assert frame.detections == ()


def test_a_target_past_the_far_clip_is_not_seen() -> None:
    # The camera model's max_range_m mirrors the simulated camera's far clip:
    # beyond it the rendered image shows nothing, so nothing may be reported.
    assert detector([car(30.0, 0.0)]).detect(pose(z=45.0), frame_index=1).visible_target_ids == ()


def test_a_target_outside_the_field_of_view_is_not_seen() -> None:
    assert detector([car(12.0, 40.0)]).detect(pose(), frame_index=1).visible_target_ids == ()


def test_yaw_turns_the_camera_onto_a_target() -> None:
    north = car(0.0, 25.0)
    assert detector([north]).detect(pose(yaw_deg=0.0), frame_index=1).visible_target_ids == ()
    assert detector([north]).detect(pose(yaw_deg=90.0), frame_index=1).visible_target_ids == ("car_01",)


def test_a_building_between_camera_and_target_hides_it() -> None:
    target = car(20.0, 0.0)
    assert detector([target]).detect(pose(), frame_index=1).visible_target_ids == ("car_01",)
    # A tall wall across the line of sight, spanning the whole ray height.
    wall = OcclusionMesh(np.array([
        [[10.0, -20.0, -5.0], [10.0, 20.0, -5.0], [10.0, 20.0, 40.0]],
        [[10.0, -20.0, -5.0], [10.0, 20.0, 40.0], [10.0, -20.0, 40.0]],
    ]))
    blocked = detector([target], mesh=wall)
    assert blocked.occlusion_enabled
    assert blocked.detect(pose(), frame_index=1).visible_target_ids == ()


def test_a_target_too_small_to_resolve_is_not_reported() -> None:
    target = car(12.0, 0.0)
    assert detector([target], min_pixel_size=4.0).detect(pose(), frame_index=1).detections
    assert not detector([target], min_pixel_size=400.0).detect(pose(), frame_index=1).detections


def test_a_mostly_cropped_target_is_not_reported() -> None:
    # At this offset two thirds of the box is inside the image and the rest runs
    # off the left edge, so the visibility threshold decides whether it counts.
    target = car(12.0, 22.0)
    assert detector([target], min_visible_fraction=0.6).detect(pose(), frame_index=1).detections
    assert not detector([target], min_visible_fraction=0.9).detect(pose(), frame_index=1).detections


def test_missed_detections_leave_the_target_geometrically_visible() -> None:
    """A statistical miss is reported differently from a geometric one."""
    never = perfect_error_model(
        miss=MissModel(half_probability_px=1e6, slope_px=1.0, max_probability=1.0))
    frame = detector([car(12.0, 0.0)], error_model=never).detect(pose(), frame_index=1)
    assert frame.visible_target_ids == ("car_01",)
    assert frame.detections == ()


def test_false_positives_have_no_ground_truth_target() -> None:
    error_model = perfect_error_model(
        false_positives=FalsePositiveModel(per_frame_rate=5.0, classes=("van",)))
    frame = detector([], error_model=error_model).detect(pose(), frame_index=7)
    assert frame.detections
    for detection in frame.detections:
        assert detection.target_id is None
        assert detection.range_m is None
        assert detection.class_id == "van"
        assert detection.detection_id.endswith("#000007")


def test_a_frame_is_published_after_its_inference_latency() -> None:
    error_model = perfect_error_model(latency=LatencyModel(mean_sec=0.05, jitter_sec=0.0))
    frame = detector([car(12.0, 0.0)], error_model=error_model).detect(
        pose(stamp=1000.0), frame_index=1)
    assert frame.stamp_sec == 1000.0
    assert frame.publish_at_sec == pytest.approx(1000.05)


def test_low_scoring_detections_are_filtered_like_a_real_threshold() -> None:
    error_model = perfect_error_model(
        score=ScoreModel(min_score=0.1, max_score=0.1, sigma=0.0, publish_threshold=0.3))
    frame = detector([car(12.0, 0.0)], error_model=error_model).detect(pose(), frame_index=1)
    assert frame.visible_target_ids == ("car_01",)
    assert frame.detections == ()


def test_the_same_seed_replays_the_same_frames() -> None:
    def run(seed: int):
        error_model = DetectionErrorModel(ErrorModelConfig(), seed=seed)
        mock = detector([car(12.0, 0.0), car(16.0, -6.0, "car_02")], error_model=error_model)
        return [
            [(d.detection_id, round(d.score, 9), round(d.box.center_x, 9))
             for d in mock.detect(pose(x=index * 0.1), frame_index=index).detections]
            for index in range(1, 30)
        ]

    assert run(42) == run(42)
    assert run(42) != run(43)


def test_shipped_targets_are_not_visible_from_the_takeoff_hover() -> None:
    """A search must have to fly to find the shipped targets.

    Targets within sight of the launch point get confirmed before the vehicle has
    moved, which makes a coverage search prove nothing. The camera reaches about
    35 m ahead and 48 m at the image corners from the takeoff hover, so every
    shipped target is placed beyond that; this keeps it that way when the target
    list is edited.
    """
    from detection.config import DetectionConfig, MockDetectorConfig, load_targets

    config_dir = Path(__file__).resolve().parents[3] / "config"
    detection = DetectionConfig.load(config_dir / "detection.yaml")
    mock_config = MockDetectorConfig.load(config_dir / "mock_detector.yaml")
    field = load_targets(config_dir / "targets.yaml", ground_z_m=detection.world.ground_z_m)
    mock = MockDetector(
        camera=detection.camera, camera_extrinsics=detection.camera_extrinsics,
        targets=field, error_model=perfect_error_model(),
        visibility=mock_config.visibility, occlusion_mesh=None)

    visible: set[str] = set()
    # The offboard FSM hovers at its configured takeoff height; sweep a range of
    # heights and every heading, because the vehicle may hold any yaw.
    for hover_z in (9.0, 10.0, 11.0, 15.0):
        for yaw_deg in range(0, 360, 5):
            frame = mock.detect(pose(z=hover_z, yaw_deg=float(yaw_deg)), frame_index=1)
            visible.update(frame.visible_target_ids)
    assert not visible, f"visible from the launch hover: {', '.join(sorted(visible))}"
