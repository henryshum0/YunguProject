from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from skills import SkillConfigError, SkillRuntimeConfig


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
OFFBOARD_CONFIG = WORKSPACE_ROOT / "src" / "navigation" / "config" / "offboard"
PLANNER_CONFIG = WORKSPACE_ROOT / "src" / "search" / "config" / "yungu_planner.json"


def test_runtime_config_loads_workspace_defaults() -> None:
    config = SkillRuntimeConfig.load(OFFBOARD_CONFIG, PLANNER_CONFIG)
    assert config.offboard.frame_id == "map"
    assert config.offboard.queue_service == "/waypoint_buffer"
    assert config.offboard.clear_service == "/waypoint_buffer/clear"
    assert config.offboard.takeoff_topic == "/takeoff_cmd"
    assert config.offboard.land_topic == "/land_cmd"
    assert config.offboard.queue_status_topic == "/waypoint_buffer/status"
    assert config.coverage_planner.frame_id == "map"
    assert config.coverage_planner.plan_service == "/coverage_planner/plan_coverage"
    assert config.coverage_planner.planner_config_file == PLANNER_CONFIG.resolve()


def test_runtime_config_rejects_missing_and_invalid_package_configs(tmp_path: Path) -> None:
    with pytest.raises(SkillConfigError, match="offboard FSM config file does not exist"):
        SkillRuntimeConfig.load(tmp_path, PLANNER_CONFIG)

    _write_offboard_configs(tmp_path, queue_name="not_absolute")
    with pytest.raises(SkillConfigError, match="queue_waypoints must be an absolute"):
        SkillRuntimeConfig.load(tmp_path, PLANNER_CONFIG)

    malformed = tmp_path / "planner.json"
    malformed.write_text("{invalid", encoding="utf-8")
    with pytest.raises(SkillConfigError, match="invalid coverage planner config"):
        SkillRuntimeConfig.load(OFFBOARD_CONFIG, malformed)


def test_runtime_config_rejects_missing_topic_keys_and_frame_mismatch(tmp_path: Path) -> None:
    _write_offboard_configs(tmp_path, frame_id="world", omit_status=True)
    with pytest.raises(SkillConfigError, match="waypoint_queue_status is required"):
        SkillRuntimeConfig.load(tmp_path, PLANNER_CONFIG)

    _write_offboard_configs(tmp_path, frame_id="world")
    with pytest.raises(SkillConfigError, match="does not match coverage planner frame_id"):
        SkillRuntimeConfig.load(tmp_path, PLANNER_CONFIG)


def _write_offboard_configs(
    directory: Path,
    *,
    frame_id: str = "map",
    queue_name: str = "/waypoint_buffer",
    omit_status: bool = False,
) -> None:
    directory.mkdir(exist_ok=True)
    (directory / "offboard_fsm.yaml").write_text(
        yaml.safe_dump({"offboard_fsm": {"frame_id": frame_id}}), encoding="utf-8")
    outgoing = {} if omit_status else {"waypoint_queue_status": "/waypoint_buffer/status"}
    (directory / "topics.yaml").write_text(yaml.safe_dump({
        "offboard_fsm": {
            "services": {"queue_waypoints": queue_name, "clear_waypoints": "/waypoint_buffer/clear"},
            "in": {"takeoff_cmd": "/takeoff_cmd", "land_cmd": "/land_cmd"},
            "out": outgoing,
        },
    }), encoding="utf-8")


DETECTION_CONFIG = WORKSPACE_ROOT / "src" / "detection" / "config" / "detection.yaml"


def test_detection_is_optional_and_absent_by_default() -> None:
    config = SkillRuntimeConfig.load(OFFBOARD_CONFIG, PLANNER_CONFIG)
    assert config.detection is None


def test_runtime_config_loads_the_workspace_detection_contract() -> None:
    config = SkillRuntimeConfig.load(OFFBOARD_CONFIG, PLANNER_CONFIG, DETECTION_CONFIG)
    assert config.detection is not None
    assert config.detection.frame_id == config.offboard.frame_id
    assert config.detection.detections_topic == "/detection/detections"
    assert config.detection.detections_world_topic == "/detection/detections_world"
    assert config.detection.image_overlay_topic == "/detection/image_overlay"
    assert config.detection.vehicle_odom_topic == "/gz/odom_super"
    assert config.detection.config_file == DETECTION_CONFIG.resolve()


def test_a_detection_frame_that_disagrees_with_navigation_is_rejected(tmp_path: Path) -> None:
    payload = yaml.safe_load(DETECTION_CONFIG.read_text(encoding="utf-8"))
    payload["world"]["frame_id"] = "odom"
    mismatched = tmp_path / "detection.yaml"
    mismatched.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(SkillConfigError, match="does not match offboard_fsm.frame_id"):
        SkillRuntimeConfig.load(OFFBOARD_CONFIG, PLANNER_CONFIG, mismatched)


def test_a_missing_detection_config_is_reported(tmp_path: Path) -> None:
    with pytest.raises(SkillConfigError, match="detection config file does not exist"):
        SkillRuntimeConfig.load(OFFBOARD_CONFIG, PLANNER_CONFIG, tmp_path / "nothing.yaml")
