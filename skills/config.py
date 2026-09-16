"""Validated workspace configuration for ROS-backed skills interfaces."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


class SkillConfigError(ValueError):
    """Raised when a package configuration cannot safely drive a skill."""


@dataclass(frozen=True, slots=True)
class OffboardSkillConfig:
    """Skills-relevant endpoints declared by the offboard FSM configuration."""

    frame_id: str
    queue_service: str
    clear_service: str
    takeoff_service: str
    land_service: str
    queue_status_topic: str


@dataclass(frozen=True, slots=True)
class CoveragePlannerSkillConfig:
    """Skills-relevant coverage planner settings parsed from its startup JSON."""

    frame_id: str
    plan_service: str
    planner_config_file: Path


@dataclass(frozen=True, slots=True)
class DetectionSkillConfig:
    """Skills-relevant endpoints declared by the detection subsystem contract."""

    frame_id: str
    detections_topic: str
    detections_world_topic: str
    image_overlay_topic: str
    vehicle_odom_topic: str
    config_file: Path


@dataclass(frozen=True, slots=True)
class SkillRuntimeConfig:
    """One immutable configuration source for all navigation/search skills."""

    offboard: OffboardSkillConfig
    coverage_planner: CoveragePlannerSkillConfig
    #: Present only when a detection configuration was supplied; skills that need
    #: detections raise a configuration error when it is missing.
    detection: DetectionSkillConfig | None = None

    @classmethod
    def load(
        cls,
        navigation_config_dir: str | Path,
        planner_config_file: str | Path,
        detection_config_file: str | Path | None = None,
    ) -> "SkillRuntimeConfig":
        """Load offboard YAML, coverage-planner JSON, and optional detection YAML.

        ``detection_config_file`` is optional so navigation-only and search-only
        applications keep working unchanged; it is required by ``DetectSkill``
        and by the full ``SearchMissionSkill`` mission.
        """
        config_dir = Path(navigation_config_dir).expanduser().resolve()
        fsm_file = config_dir / "offboard_fsm.yaml"
        topics_file = config_dir / "topics.yaml"
        fsm = _load_yaml_mapping(fsm_file, "offboard FSM")
        topics = _load_yaml_mapping(topics_file, "offboard topics")
        offboard = _parse_offboard(fsm, topics)

        planner_file = Path(planner_config_file).expanduser().resolve()
        if not planner_file.is_file():
            raise SkillConfigError(f"coverage planner config file does not exist: '{planner_file}'")
        try:
            from coverage_planner.io import ConfigError, load_config
        except ImportError as exc:
            raise SkillConfigError(
                "coverage_planner is unavailable; source the built workspace overlay before "
                "loading skill configuration") from exc
        try:
            planner = load_config(planner_file)
        except ConfigError as exc:
            raise SkillConfigError(f"invalid coverage planner config '{planner_file}': {exc}") from exc
        coverage = CoveragePlannerSkillConfig(
            frame_id=_frame_id(planner.frame_id, "coverage planner frame_id"),
            plan_service="/coverage_planner/plan_coverage",
            planner_config_file=planner_file,
        )
        if offboard.frame_id != coverage.frame_id:
            raise SkillConfigError(
                "offboard_fsm.frame_id "
                f"'{offboard.frame_id}' does not match coverage planner frame_id "
                f"'{coverage.frame_id}'")

        detection = None
        if detection_config_file is not None:
            detection = _load_detection(detection_config_file)
            if detection.frame_id != offboard.frame_id:
                raise SkillConfigError(
                    f"detection world frame_id '{detection.frame_id}' does not match "
                    f"offboard_fsm.frame_id '{offboard.frame_id}'")
        return cls(offboard=offboard, coverage_planner=coverage, detection=detection)


def _load_detection(detection_config_file: str | Path) -> DetectionSkillConfig:
    """Validate the detection contract through the detection package's own loader."""
    config_file = Path(detection_config_file).expanduser().resolve()
    if not config_file.is_file():
        raise SkillConfigError(f"detection config file does not exist: '{config_file}'")
    try:
        from detection.config import DetectionConfig, DetectionConfigError
    except ImportError as exc:
        raise SkillConfigError(
            "the detection package is unavailable; source the built workspace overlay "
            "before loading a detection-aware skill configuration") from exc
    try:
        config = DetectionConfig.load(config_file)
    except DetectionConfigError as exc:
        raise SkillConfigError(f"invalid detection config '{config_file}': {exc}") from exc
    return DetectionSkillConfig(
        frame_id=config.world.frame_id,
        detections_topic=config.topics.detections,
        detections_world_topic=config.topics.detections_world,
        image_overlay_topic=config.topics.image_overlay,
        vehicle_odom_topic=config.topics.vehicle_odom,
        config_file=config_file,
    )


def _load_yaml_mapping(path: Path, label: str) -> Mapping[str, Any]:
    if not path.is_file():
        raise SkillConfigError(f"{label} config file does not exist: '{path}'")
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SkillConfigError(f"cannot read {label} config '{path}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise SkillConfigError(f"invalid YAML in {label} config '{path}': {exc}") from exc
    if not isinstance(payload, Mapping):
        raise SkillConfigError(f"{label} config '{path}' must contain a YAML mapping")
    return payload


def _parse_offboard(fsm_payload: Mapping[str, Any], topics_payload: Mapping[str, Any]) -> OffboardSkillConfig:
    fsm = _mapping(fsm_payload, "offboard_fsm")
    offboard_topics = _mapping(topics_payload, "offboard_fsm")
    services = _mapping(offboard_topics, "services", parent="offboard_fsm")
    outgoing = _mapping(offboard_topics, "out", parent="offboard_fsm")
    return OffboardSkillConfig(
        frame_id=_frame_id(_required(fsm, "frame_id", "offboard_fsm"), "offboard_fsm.frame_id"),
        queue_service=_ros_name(_required(services, "queue_waypoints", "offboard_fsm.services"),
                                "offboard_fsm.services.queue_waypoints"),
        clear_service=_ros_name(_required(services, "clear_waypoints", "offboard_fsm.services"),
                                "offboard_fsm.services.clear_waypoints"),
        takeoff_service=_ros_name(_required(services, "takeoff", "offboard_fsm.services"),
                                  "offboard_fsm.services.takeoff"),
        land_service=_ros_name(_required(services, "land", "offboard_fsm.services"),
                               "offboard_fsm.services.land"),
        queue_status_topic=_ros_name(_required(outgoing, "waypoint_queue_status", "offboard_fsm.out"),
                                     "offboard_fsm.out.waypoint_queue_status"),
    )


def _mapping(data: Mapping[str, Any], key: str, *, parent: str = "") -> Mapping[str, Any]:
    path = f"{parent}.{key}" if parent else key
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise SkillConfigError(f"{path} must be a mapping")
    return value


def _required(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise SkillConfigError(f"{path}.{key} is required")
    return data[key]


def _frame_id(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip() or any(char.isspace() for char in value):
        raise SkillConfigError(f"{path} must be a non-empty frame ID without whitespace")
    return value


def _ros_name(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or value == "/" or "//" in value:
        raise SkillConfigError(f"{path} must be an absolute non-root ROS name")
    if any(char.isspace() for char in value):
        raise SkillConfigError(f"{path} must not contain whitespace")
    return value
