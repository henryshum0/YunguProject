"""The statistical behaviour that makes mock detections look like real ones."""

from __future__ import annotations

from math import isclose

import pytest

from detection.camera import PixelBox
from detection.error_model import (
    DetectionErrorModel,
    ErrorModelConfig,
    ErrorModelError,
    FalsePositiveModel,
    LatencyModel,
    MissModel,
    ScoreModel,
)


def model(seed: int = 11, **overrides) -> DetectionErrorModel:
    return DetectionErrorModel(ErrorModelConfig(**overrides), seed=seed)


def test_recall_rises_with_target_pixel_size() -> None:
    miss = MissModel(half_probability_px=12.0, slope_px=4.0, max_probability=0.95)
    assert isclose(miss.detection_probability(12.0), 0.475, abs_tol=1e-6)
    assert miss.detection_probability(4.0) < miss.detection_probability(12.0)
    assert miss.detection_probability(60.0) < miss.max_probability
    assert miss.detection_probability(1000.0) <= miss.max_probability
    # A vanishingly small target saturates instead of overflowing the exponential.
    assert miss.detection_probability(-1e6) == 0.0


def test_detection_rate_matches_the_configured_recall() -> None:
    error_model = model()
    hits = sum(error_model.is_detected(12.0) for _ in range(20000))
    assert isclose(hits / 20000, 0.475, abs_tol=0.02)


def test_false_positive_count_matches_the_configured_rate() -> None:
    error_model = model(false_positives=FalsePositiveModel(per_frame_rate=0.2))
    total = sum(len(error_model.sample_false_positives(640, 480)) for _ in range(5000))
    assert isclose(total / 5000, 0.2, abs_tol=0.03)


def test_false_positives_stay_inside_the_image() -> None:
    error_model = model(false_positives=FalsePositiveModel(per_frame_rate=5.0))
    for _ in range(50):
        for _class_id, box, score in error_model.sample_false_positives(640, 480):
            assert 0.0 <= box.center_x <= 640.0
            assert 0.0 <= box.center_y <= 480.0
            assert 0.30 <= score <= 0.55


def test_confidence_grows_with_pixel_size() -> None:
    score = ScoreModel(min_score=0.45, max_score=0.92, score_min_px=8.0, score_max_px=60.0)
    assert score.mean_score(4.0) == 0.45      # clamped below the range
    assert score.mean_score(200.0) == 0.92    # clamped above the range
    assert score.mean_score(8.0) < score.mean_score(34.0) < score.mean_score(60.0)
    assert 0.0 <= model().sample_score(30.0) <= 1.0


def test_class_confusion_follows_its_probabilities() -> None:
    error_model = model(confusion={"car": {"van": 0.25}})
    reported = [error_model.reported_class("car") for _ in range(10000)]
    assert isclose(reported.count("van") / 10000, 0.25, abs_tol=0.02)
    assert error_model.reported_class("bus") == "bus"  # no entry means no confusion


def test_confusion_probabilities_above_one_are_rejected() -> None:
    with pytest.raises(ErrorModelError, match="sum to"):
        ErrorModelConfig(confusion={"car": {"van": 0.7, "truck": 0.5}})


def test_box_jitter_stays_close_and_keeps_a_positive_size() -> None:
    error_model = model()
    box = PixelBox.from_center(320.0, 240.0, 40.0, 20.0)
    for _ in range(200):
        noisy = error_model.jitter_box(box)
        assert abs(noisy.center_x - box.center_x) < 10.0
        assert noisy.size_x >= 1.0 and noisy.size_y >= 1.0


def test_latency_is_non_negative_and_centred_on_its_mean() -> None:
    error_model = model(latency=LatencyModel(mean_sec=0.045, jitter_sec=0.010))
    samples = [error_model.sample_latency() for _ in range(5000)]
    assert min(samples) >= 0.0
    assert isclose(sum(samples) / len(samples), 0.045, abs_tol=0.002)
    assert DetectionErrorModel(
        ErrorModelConfig(latency=LatencyModel(mean_sec=0.02, jitter_sec=0.0)),
        seed=1).sample_latency() == 0.02


def test_a_seed_replays_the_same_draws() -> None:
    def draws(seed: int) -> list[float]:
        error_model = model(seed=seed)
        return [error_model.sample_score(20.0) for _ in range(50)]

    assert draws(5) == draws(5)
    assert draws(5) != draws(6)


def test_invalid_parameters_are_rejected() -> None:
    with pytest.raises(ErrorModelError, match="positive"):
        MissModel(slope_px=0.0)
    with pytest.raises(ErrorModelError, match="probability"):
        MissModel(max_probability=1.5)
    with pytest.raises(ErrorModelError, match="not be empty"):
        FalsePositiveModel(classes=())
    with pytest.raises(ErrorModelError, match="cannot exceed"):
        ScoreModel(min_score=0.9, max_score=0.5)
