"""Validated configuration for the detection subsystem.

Three files, each with one owner:

``detection.yaml``
    The contract every part of the detection skill shares: world frame and
    ground plane, camera model, topic names, localizer settings. A real detector
    replacing the mock keeps using this file.
``mock_detector.yaml``
    Simulation-only behaviour of the mock detector, above all the error model.
``targets.yaml``
    The simulated ground-truth targets (see :mod:`detection.targets`).

Everything is validated before any ROS client or node is created, so a bad value
fails at startup with a specific message instead of producing silently wrong
detections.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any

import yaml

from detection.camera import Extrinsics, PinholeCamera
from detection.error_model import (
    ErrorModelConfig,
    ErrorModelError,
    FalsePositiveModel,
    LatencyModel,
    LocalizationNoiseModel,
    MissModel,
    ScoreModel,
)
from detection.targets import TargetConfigError, TargetField, parse_targets


class DetectionConfigError(ValueError):
    """Raised when a detection configuration file cannot safely drive a node."""


ANCHORS = ("bottom_center", "center")


@dataclass(frozen=True, slots=True)
class WorldConfig:
    """The ENU frame every world-frame detection is expressed in."""

    frame_id: str
    ground_z_m: float


@dataclass(frozen=True, slots=True)
class TopicsConfig:
    """Every ROS endpoint of the detection subsystem."""

    detections: str
    detections_world: str
    markers: str
    vehicle_odom: str
    image_in: str
    image_overlay: str


@dataclass(frozen=True, slots=True)
class LocalizerConfig:
    """Settings of the image-plane to ground-plane back-projection."""

    anchor: str
    odom_buffer_sec: float
    max_pose_age_sec: float
    max_range_m: float


@dataclass(frozen=True, slots=True)
class OverlayConfig:
    """The debug image overlay."""

    enabled: bool
    max_rate_hz: float


@dataclass(frozen=True, slots=True)
class DetectionConfig:
    """The shared detection contract loaded from ``detection.yaml``."""

    world: WorldConfig
    camera: PinholeCamera
    camera_extrinsics: Extrinsics
    topics: TopicsConfig
    localizer: LocalizerConfig
    overlay: OverlayConfig

    @classmethod
    def load(cls, path: str | Path) -> "DetectionConfig":
        payload = _load_yaml_mapping(path, "detection")
        world = _mapping(payload, "world")
        camera = _mapping(payload, "camera")
        topics = _mapping(payload, "topics")
        localizer = _mapping(payload, "localizer")
        overlay = _mapping(payload, "overlay")

        try:
            pinhole = PinholeCamera.from_horizontal_fov(
                width=_positive_int(camera, "width", "camera"),
                height=_positive_int(camera, "height", "camera"),
                horizontal_fov_deg=_number(camera, "horizontal_fov_deg", "camera"),
                max_range_m=_number(camera, "max_range_m", "camera"),
                frame_id=_frame_id(_required(camera, "frame_id", "camera"), "camera.frame_id"),
            )
            extrinsics = Extrinsics.from_rpy(
                _vector3(camera, "translation", "camera"),
                *_vector3(camera, "rotation_rpy_deg", "camera"),
            )
        except ValueError as error:
            raise DetectionConfigError(f"invalid camera configuration: {error}") from error

        anchor = str(_required(localizer, "anchor", "localizer"))
        if anchor not in ANCHORS:
            raise DetectionConfigError(
                f"localizer.anchor must be one of {', '.join(ANCHORS)}, got '{anchor}'")

        return cls(
            world=WorldConfig(
                frame_id=_frame_id(_required(world, "frame_id", "world"), "world.frame_id"),
                ground_z_m=_number(world, "ground_z_m", "world"),
            ),
            camera=pinhole,
            camera_extrinsics=extrinsics,
            topics=TopicsConfig(
                detections=_ros_name(topics, "detections", "topics"),
                detections_world=_ros_name(topics, "detections_world", "topics"),
                markers=_ros_name(topics, "markers", "topics"),
                vehicle_odom=_ros_name(topics, "vehicle_odom", "topics"),
                image_in=_ros_name(topics, "image_in", "topics"),
                image_overlay=_ros_name(topics, "image_overlay", "topics"),
            ),
            localizer=LocalizerConfig(
                anchor=anchor,
                odom_buffer_sec=_positive(localizer, "odom_buffer_sec", "localizer"),
                max_pose_age_sec=_positive(localizer, "max_pose_age_sec", "localizer"),
                max_range_m=_positive(localizer, "max_range_m", "localizer"),
            ),
            overlay=OverlayConfig(
                enabled=_boolean(overlay, "enabled", "overlay"),
                max_rate_hz=_positive(overlay, "max_rate_hz", "overlay"),
            ),
        )


@dataclass(frozen=True, slots=True)
class OcclusionConfig:
    """Line-of-sight checking against the world collision mesh."""

    enabled: bool
    collision_mesh: str
    world_origin_in_gz: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class VisibilityConfig:
    """Geometric gates applied before the error model."""

    min_pixel_size: float
    min_visible_fraction: float
    occlusion: OcclusionConfig


@dataclass(frozen=True, slots=True)
class MockDetectorConfig:
    """Simulation-only mock detector settings loaded from ``mock_detector.yaml``."""

    rate_hz: float
    seed: int | None
    visibility: VisibilityConfig
    error_model: ErrorModelConfig

    @classmethod
    def load(cls, path: str | Path) -> "MockDetectorConfig":
        payload = _load_yaml_mapping(path, "mock detector")
        mock = _mapping(payload, "mock_detector")
        visibility = _mapping(mock, "visibility", parent="mock_detector")
        occlusion = _mapping(visibility, "occlusion", parent="mock_detector.visibility")
        errors = _mapping(mock, "error_model", parent="mock_detector")

        seed = mock.get("seed")
        if seed is not None and not isinstance(seed, int):
            raise DetectionConfigError("mock_detector.seed must be an integer or null")

        fraction = _number(visibility, "min_visible_fraction", "mock_detector.visibility")
        if not 0.0 < fraction <= 1.0:
            raise DetectionConfigError(
                "mock_detector.visibility.min_visible_fraction must be in (0, 1]")

        try:
            error_model = _parse_error_model(errors)
        except ErrorModelError as error:
            raise DetectionConfigError(f"invalid mock_detector.error_model: {error}") from error

        return cls(
            rate_hz=_positive(mock, "rate_hz", "mock_detector"),
            seed=seed,
            visibility=VisibilityConfig(
                min_pixel_size=_positive(visibility, "min_pixel_size", "mock_detector.visibility"),
                min_visible_fraction=fraction,
                occlusion=OcclusionConfig(
                    enabled=_boolean(occlusion, "enabled", "mock_detector.visibility.occlusion"),
                    collision_mesh=str(_required(
                        occlusion, "collision_mesh", "mock_detector.visibility.occlusion")),
                    world_origin_in_gz=_vector3(
                        occlusion, "world_origin_in_gz", "mock_detector.visibility.occlusion"),
                ),
            ),
            error_model=error_model,
        )


def load_targets(path: str | Path, *, ground_z_m: float) -> TargetField:
    """Load ``targets.yaml``, placing targets without an explicit z on the ground."""
    payload = _load_yaml_mapping(path, "targets")
    try:
        return parse_targets(payload, ground_z_m=ground_z_m)
    except TargetConfigError as error:
        raise DetectionConfigError(f"invalid targets config '{path}': {error}") from error


def _parse_error_model(payload: Mapping[str, Any]) -> ErrorModelConfig:
    miss = _mapping(payload, "miss", parent="error_model")
    false_positives = _mapping(payload, "false_positives", parent="error_model")
    localization = _mapping(payload, "localization", parent="error_model")
    score = _mapping(payload, "score", parent="error_model")
    latency = _mapping(payload, "latency", parent="error_model")
    confusion = payload.get("confusion") or {}
    if not isinstance(confusion, Mapping):
        raise DetectionConfigError("error_model.confusion must be a mapping")

    classes = false_positives.get("classes")
    if not isinstance(classes, Sequence) or isinstance(classes, (str, bytes)) or not classes:
        raise DetectionConfigError("error_model.false_positives.classes must be a non-empty list")

    return ErrorModelConfig(
        miss=MissModel(
            half_probability_px=float(miss["half_probability_px"]),
            slope_px=float(miss["slope_px"]),
            max_probability=float(miss["max_probability"]),
        ),
        false_positives=FalsePositiveModel(
            per_frame_rate=float(false_positives["per_frame_rate"]),
            classes=tuple(str(name) for name in classes),
            score_range=_pair(false_positives, "score_range", "error_model.false_positives"),
            size_px_range=_pair(false_positives, "size_px_range", "error_model.false_positives"),
        ),
        localization=LocalizationNoiseModel(
            center_sigma_ratio=float(localization["center_sigma_ratio"]),
            center_sigma_floor_px=float(localization["center_sigma_floor_px"]),
            size_sigma_ratio=float(localization["size_sigma_ratio"]),
            size_sigma_floor_px=float(localization["size_sigma_floor_px"]),
        ),
        score=ScoreModel(
            min_score=float(score["min_score"]),
            max_score=float(score["max_score"]),
            score_min_px=float(score["score_min_px"]),
            score_max_px=float(score["score_max_px"]),
            sigma=float(score["sigma"]),
            publish_threshold=float(score["publish_threshold"]),
        ),
        confusion={
            str(true_class): {str(name): float(value) for name, value in alternatives.items()}
            for true_class, alternatives in confusion.items()
        },
        latency=LatencyModel(
            mean_sec=float(latency["mean_sec"]),
            jitter_sec=float(latency["jitter_sec"]),
        ),
    )


def _load_yaml_mapping(path: str | Path, label: str) -> Mapping[str, Any]:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise DetectionConfigError(f"{label} config file does not exist: '{config_path}'")
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise DetectionConfigError(f"cannot read {label} config '{config_path}': {error}") from error
    except yaml.YAMLError as error:
        raise DetectionConfigError(f"invalid YAML in {label} config '{config_path}': {error}") from error
    if not isinstance(payload, Mapping):
        raise DetectionConfigError(f"{label} config '{config_path}' must contain a YAML mapping")
    return payload


def _mapping(data: Mapping[str, Any], key: str, *, parent: str = "") -> Mapping[str, Any]:
    path = f"{parent}.{key}" if parent else key
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise DetectionConfigError(f"{path} must be a mapping")
    return value


def _required(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise DetectionConfigError(f"{path}.{key} is required")
    return data[key]


def _number(data: Mapping[str, Any], key: str, path: str) -> float:
    value = _required(data, key, path)
    if isinstance(value, bool):
        raise DetectionConfigError(f"{path}.{key} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise DetectionConfigError(f"{path}.{key} must be a finite number") from error
    if not isfinite(number):
        raise DetectionConfigError(f"{path}.{key} must be a finite number")
    return number


def _positive(data: Mapping[str, Any], key: str, path: str) -> float:
    number = _number(data, key, path)
    if number <= 0.0:
        raise DetectionConfigError(f"{path}.{key} must be positive")
    return number


def _positive_int(data: Mapping[str, Any], key: str, path: str) -> int:
    number = _number(data, key, path)
    if number <= 0 or number != int(number):
        raise DetectionConfigError(f"{path}.{key} must be a positive integer")
    return int(number)


def _boolean(data: Mapping[str, Any], key: str, path: str) -> bool:
    value = _required(data, key, path)
    if not isinstance(value, bool):
        raise DetectionConfigError(f"{path}.{key} must be true or false")
    return value


def _vector3(data: Mapping[str, Any], key: str, path: str) -> tuple[float, float, float]:
    value = _required(data, key, path)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise DetectionConfigError(f"{path}.{key} must be three numbers")
    numbers = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError) as error:
            raise DetectionConfigError(f"{path}.{key} must be three finite numbers") from error
        if not isfinite(number):
            raise DetectionConfigError(f"{path}.{key} must be three finite numbers")
        numbers.append(number)
    return (numbers[0], numbers[1], numbers[2])


def _pair(data: Mapping[str, Any], key: str, path: str) -> tuple[float, float]:
    value = _required(data, key, path)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise DetectionConfigError(f"{path}.{key} must be two numbers")
    return (float(value[0]), float(value[1]))


def _frame_id(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip() or any(char.isspace() for char in value):
        raise DetectionConfigError(f"{path} must be a non-empty frame ID without whitespace")
    return value


def _ros_name(data: Mapping[str, Any], key: str, path: str) -> str:
    value = _required(data, key, path)
    if not isinstance(value, str) or not value.startswith("/") or value == "/" or "//" in value:
        raise DetectionConfigError(f"{path}.{key} must be an absolute non-root ROS name")
    if any(char.isspace() for char in value):
        raise DetectionConfigError(f"{path}.{key} must not contain whitespace")
    return value


def resolve_workspace_path(value: str | Path, *, reference: str | Path) -> Path:
    """Resolve a possibly-relative configured path against the workspace layout.

    Absolute paths are returned unchanged. A relative path is searched for
    upwards from the directory holding ``reference`` (normally the configuration
    file that declared it), which finds workspace-relative resources such as the
    Gazebo collision mesh without hard-coding an installation prefix.
    """
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate
    base = Path(reference).expanduser().resolve()
    start = base if base.is_dir() else base.parent
    for parent in [start, *start.parents]:
        resolved = parent / candidate
        if resolved.exists():
            return resolved
    return start / candidate
