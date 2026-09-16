"""Statistical error model that turns exact projections into detector-like output.

Geometric projection alone would give a perfect oracle: every target, every
frame, pixel-exact. A real detector misses small targets, invents boxes, jitters
corners, is unsure about the class, and answers late. Reproducing that behaviour
is what makes a simulated search decision meaningful, because the search logic
then faces the same statistics it will face on the vehicle.

Every parameter here is calibrated from the real detector measured on real data
(recall versus target size, per-frame false-positive rate, localization
variance, score distribution, inference latency). The defaults shipped in
``src/detection/config/mock_detector.yaml`` are provisional placeholders.

All randomness comes from one seeded ``random.Random``, so a run replays exactly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import exp, isfinite
import random
from typing import Mapping, Sequence

from detection.camera import PixelBox


class ErrorModelError(ValueError):
    """Raised when an error-model parameter cannot describe a valid distribution."""


def _positive(value: float, name: str) -> float:
    number = float(value)
    if not isfinite(number) or number <= 0.0:
        raise ErrorModelError(f"{name} must be a positive finite number")
    return number


def _non_negative(value: float, name: str) -> float:
    number = float(value)
    if not isfinite(number) or number < 0.0:
        raise ErrorModelError(f"{name} must be a non-negative finite number")
    return number


def _probability(value: float, name: str) -> float:
    number = float(value)
    if not isfinite(number) or not 0.0 <= number <= 1.0:
        raise ErrorModelError(f"{name} must be a probability in [0, 1]")
    return number


@dataclass(frozen=True, slots=True)
class MissModel:
    """Recall as a logistic function of the projected box's short side."""

    half_probability_px: float = 12.0
    slope_px: float = 4.0
    max_probability: float = 0.95

    def __post_init__(self) -> None:
        _positive(self.half_probability_px, "miss.half_probability_px")
        _positive(self.slope_px, "miss.slope_px")
        _probability(self.max_probability, "miss.max_probability")

    def detection_probability(self, short_side_px: float) -> float:
        """Probability that a target this many pixels across is reported at all."""
        exponent = (self.half_probability_px - float(short_side_px)) / self.slope_px
        # Guard the exponential so a tiny target saturates instead of overflowing.
        if exponent > 700.0:
            return 0.0
        return self.max_probability / (1.0 + exp(exponent))


@dataclass(frozen=True, slots=True)
class FalsePositiveModel:
    """Spurious boxes injected per published frame."""

    per_frame_rate: float = 0.02
    classes: tuple[str, ...] = ("pedestrian", "car")
    score_range: tuple[float, float] = (0.30, 0.55)
    size_px_range: tuple[float, float] = (8.0, 40.0)

    def __post_init__(self) -> None:
        _non_negative(self.per_frame_rate, "false_positives.per_frame_rate")
        if not self.classes:
            raise ErrorModelError("false_positives.classes must not be empty")
        for name, bounds in (("score_range", self.score_range),
                             ("size_px_range", self.size_px_range)):
            low, high = float(bounds[0]), float(bounds[1])
            if not isfinite(low) or not isfinite(high) or low > high:
                raise ErrorModelError(f"false_positives.{name} must be an ordered finite pair")
        _probability(self.score_range[0], "false_positives.score_range lower bound")
        _probability(self.score_range[1], "false_positives.score_range upper bound")
        _positive(self.size_px_range[0], "false_positives.size_px_range lower bound")


@dataclass(frozen=True, slots=True)
class LocalizationNoiseModel:
    """Gaussian jitter on the reported box centre and size."""

    center_sigma_ratio: float = 0.03
    center_sigma_floor_px: float = 0.5
    size_sigma_ratio: float = 0.05
    size_sigma_floor_px: float = 0.5

    def __post_init__(self) -> None:
        _non_negative(self.center_sigma_ratio, "localization.center_sigma_ratio")
        _non_negative(self.center_sigma_floor_px, "localization.center_sigma_floor_px")
        _non_negative(self.size_sigma_ratio, "localization.size_sigma_ratio")
        _non_negative(self.size_sigma_floor_px, "localization.size_sigma_floor_px")


@dataclass(frozen=True, slots=True)
class ScoreModel:
    """Confidence drawn around a size-dependent mean."""

    min_score: float = 0.45
    max_score: float = 0.92
    score_min_px: float = 8.0
    score_max_px: float = 60.0
    sigma: float = 0.06
    publish_threshold: float = 0.30

    def __post_init__(self) -> None:
        _probability(self.min_score, "score.min_score")
        _probability(self.max_score, "score.max_score")
        _probability(self.publish_threshold, "score.publish_threshold")
        _non_negative(self.sigma, "score.sigma")
        if self.min_score > self.max_score:
            raise ErrorModelError("score.min_score cannot exceed score.max_score")
        if not _positive(self.score_min_px, "score.score_min_px") < _positive(
                self.score_max_px, "score.score_max_px"):
            raise ErrorModelError("score.score_min_px must be smaller than score.score_max_px")

    def mean_score(self, short_side_px: float) -> float:
        """Mean confidence for a target this many pixels across."""
        span = self.score_max_px - self.score_min_px
        ratio = (float(short_side_px) - self.score_min_px) / span
        ratio = min(max(ratio, 0.0), 1.0)
        return self.min_score + ratio * (self.max_score - self.min_score)


@dataclass(frozen=True, slots=True)
class LatencyModel:
    """Inference latency the detector output is delayed by."""

    mean_sec: float = 0.045
    jitter_sec: float = 0.010

    def __post_init__(self) -> None:
        _non_negative(self.mean_sec, "latency.mean_sec")
        _non_negative(self.jitter_sec, "latency.jitter_sec")


@dataclass(frozen=True, slots=True)
class ErrorModelConfig:
    """The complete calibrated behaviour of a detector."""

    miss: MissModel = field(default_factory=MissModel)
    false_positives: FalsePositiveModel = field(default_factory=FalsePositiveModel)
    localization: LocalizationNoiseModel = field(default_factory=LocalizationNoiseModel)
    score: ScoreModel = field(default_factory=ScoreModel)
    confusion: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    latency: LatencyModel = field(default_factory=LatencyModel)

    def __post_init__(self) -> None:
        for true_class, alternatives in self.confusion.items():
            total = 0.0
            for reported, probability in alternatives.items():
                total += _probability(probability, f"confusion.{true_class}.{reported}")
            if total > 1.0:
                raise ErrorModelError(
                    f"confusion probabilities for '{true_class}' sum to {total:.3f}, above 1")


class DetectionErrorModel:
    """Apply a calibrated :class:`ErrorModelConfig` with one seeded RNG."""

    def __init__(self, config: ErrorModelConfig, *, seed: int | None = None) -> None:
        self._config = config
        self._random = random.Random(seed)

    @property
    def config(self) -> ErrorModelConfig:
        return self._config

    def is_detected(self, short_side_px: float) -> bool:
        """Draw whether a target of this pixel size is reported on this frame."""
        return self._random.random() < self._config.miss.detection_probability(short_side_px)

    def sample_score(self, short_side_px: float) -> float:
        """Draw a confidence for a target of this pixel size."""
        score = self._config.score.mean_score(short_side_px)
        if self._config.score.sigma > 0.0:
            score = self._random.gauss(score, self._config.score.sigma)
        return min(max(score, 0.0), 1.0)

    def reported_class(self, class_id: str) -> str:
        """Draw the class label reported for a true class, allowing confusion."""
        alternatives = self._config.confusion.get(class_id)
        if not alternatives:
            return class_id
        draw = self._random.random()
        cumulative = 0.0
        for reported, probability in alternatives.items():
            cumulative += probability
            if draw < cumulative:
                return reported
        return class_id

    def jitter_box(self, box: PixelBox) -> PixelBox:
        """Add localization noise to a projected box."""
        noise = self._config.localization
        center_sigma_x = max(noise.center_sigma_ratio * box.size_x, noise.center_sigma_floor_px)
        center_sigma_y = max(noise.center_sigma_ratio * box.size_y, noise.center_sigma_floor_px)
        size_sigma_x = max(noise.size_sigma_ratio * box.size_x, noise.size_sigma_floor_px)
        size_sigma_y = max(noise.size_sigma_ratio * box.size_y, noise.size_sigma_floor_px)
        center_x = box.center_x + self._random.gauss(0.0, center_sigma_x)
        center_y = box.center_y + self._random.gauss(0.0, center_sigma_y)
        size_x = max(box.size_x + self._random.gauss(0.0, size_sigma_x), 1.0)
        size_y = max(box.size_y + self._random.gauss(0.0, size_sigma_y), 1.0)
        return PixelBox.from_center(center_x, center_y, size_x, size_y)

    def sample_false_positives(self, width: int, height: int) -> tuple[tuple[str, PixelBox, float], ...]:
        """Draw this frame's spurious boxes as ``(class, box, score)`` triples."""
        model = self._config.false_positives
        spurious = []
        for _ in range(self._poisson(model.per_frame_rate)):
            size_x = self._random.uniform(*model.size_px_range)
            size_y = self._random.uniform(*model.size_px_range)
            center_x = self._random.uniform(size_x / 2.0, max(width - size_x / 2.0, size_x / 2.0))
            center_y = self._random.uniform(size_y / 2.0, max(height - size_y / 2.0, size_y / 2.0))
            spurious.append((
                self._random.choice(model.classes),
                PixelBox.from_center(center_x, center_y, size_x, size_y),
                self._random.uniform(*model.score_range),
            ))
        return tuple(spurious)

    def sample_latency(self) -> float:
        """Draw the publication delay for one frame, in seconds."""
        latency = self._config.latency
        if latency.jitter_sec <= 0.0:
            return latency.mean_sec
        return max(0.0, self._random.gauss(latency.mean_sec, latency.jitter_sec))

    def _poisson(self, rate: float) -> int:
        """Knuth's Poisson sampler; the rates here are far below 1 per frame."""
        if rate <= 0.0:
            return 0
        limit = exp(-rate)
        count = 0
        product = self._random.random()
        while product > limit:
            count += 1
            product *= self._random.random()
        return count
