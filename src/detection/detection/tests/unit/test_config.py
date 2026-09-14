"""Configuration loading, including the files the workspace actually ships."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from detection.config import (
    DetectionConfig,
    DetectionConfigError,
    MockDetectorConfig,
    load_targets,
    resolve_workspace_path,
)


CONFIG_DIR = Path(__file__).resolve().parents[3] / "config"
DETECTION_CONFIG = CONFIG_DIR / "detection.yaml"
MOCK_CONFIG = CONFIG_DIR / "mock_detector.yaml"
TARGETS_CONFIG = CONFIG_DIR / "targets.yaml"


def test_shipped_detection_config_matches_the_simulated_camera() -> None:
    config = DetectionConfig.load(DETECTION_CONFIG)
    assert config.world.frame_id == "map"  # must match offboard_fsm and the planner
    assert config.camera.width == 640 and config.camera.height == 480
    assert config.camera.fx == pytest.approx(268.51, abs=0.01)
    assert config.camera.max_range_m == 50.0  # the SDF far clip
    assert config.camera.frame_id == "front_camera_link"
    assert config.topics.detections == "/detection/detections"
    assert config.topics.vehicle_odom == "/gz/odom_super"
    assert config.localizer.anchor == "bottom_center"
    assert config.overlay.enabled and config.overlay.max_rate_hz > 0.0


def test_shipped_camera_extrinsics_pitch_the_optical_axis_down() -> None:
    config = DetectionConfig.load(DETECTION_CONFIG)
    axis = config.camera_extrinsics.direction_to_parent((1.0, 0.0, 0.0))
    assert axis[0] == pytest.approx(0.5, abs=1e-6)      # forward component
    assert axis[2] == pytest.approx(-0.866, abs=1e-3)   # 60 degrees down


def test_shipped_mock_config_loads_with_its_error_model() -> None:
    config = MockDetectorConfig.load(MOCK_CONFIG)
    assert config.rate_hz == 10.0
    assert config.seed is not None  # shipped seeded so runs replay
    assert config.visibility.occlusion.enabled
    assert not hasattr(config, "overlay")   # the overlay is not mock-specific
    assert config.error_model.miss.max_probability <= 1.0
    assert config.error_model.latency.mean_sec > 0.0
    assert "car" in config.error_model.confusion


def test_shipped_targets_stand_on_the_configured_ground() -> None:
    detection = DetectionConfig.load(DETECTION_CONFIG)
    field = load_targets(TARGETS_CONFIG, ground_z_m=detection.world.ground_z_m)
    assert field.targets
    assert {target.class_id for target in field.targets} <= {"car", "pedestrian"}
    for target in field.targets:
        assert target.position[2] == detection.world.ground_z_m
        assert target.model.startswith("yungu_")
    # The spawn offset puts them back on the Gazebo ground plane at z = -0.5.
    x, y, z = field.targets[0].gazebo_position(field.world_origin_in_gz)
    assert z == pytest.approx(-0.5, abs=0.001)


def test_missing_and_malformed_files_are_reported(tmp_path: Path) -> None:
    with pytest.raises(DetectionConfigError, match="does not exist"):
        DetectionConfig.load(tmp_path / "nothing.yaml")
    broken = tmp_path / "broken.yaml"
    broken.write_text("world: [not, a, mapping]", encoding="utf-8")
    with pytest.raises(DetectionConfigError, match="world must be a mapping"):
        DetectionConfig.load(broken)
    not_yaml = tmp_path / "bad.yaml"
    not_yaml.write_text("{unterminated", encoding="utf-8")
    with pytest.raises(DetectionConfigError, match="invalid YAML"):
        DetectionConfig.load(not_yaml)


def _write(path: Path, payload: dict) -> Path:
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return path


def test_invalid_values_are_rejected_with_their_field_name(tmp_path: Path) -> None:
    payload = yaml.safe_load(DETECTION_CONFIG.read_text(encoding="utf-8"))

    payload["topics"]["detections"] = "detection/detections"
    with pytest.raises(DetectionConfigError, match="topics.detections must be an absolute"):
        DetectionConfig.load(_write(tmp_path / "a.yaml", payload))

    payload = yaml.safe_load(DETECTION_CONFIG.read_text(encoding="utf-8"))
    payload["localizer"]["anchor"] = "top_left"
    with pytest.raises(DetectionConfigError, match="localizer.anchor must be one of"):
        DetectionConfig.load(_write(tmp_path / "b.yaml", payload))

    payload = yaml.safe_load(DETECTION_CONFIG.read_text(encoding="utf-8"))
    payload["camera"]["horizontal_fov_deg"] = 200.0
    with pytest.raises(DetectionConfigError, match="invalid camera configuration"):
        DetectionConfig.load(_write(tmp_path / "c.yaml", payload))

    mock_payload = yaml.safe_load(MOCK_CONFIG.read_text(encoding="utf-8"))
    mock_payload["mock_detector"]["visibility"]["min_visible_fraction"] = 1.5
    with pytest.raises(DetectionConfigError, match="min_visible_fraction must be in"):
        MockDetectorConfig.load(_write(tmp_path / "d.yaml", mock_payload))

    mock_payload = yaml.safe_load(MOCK_CONFIG.read_text(encoding="utf-8"))
    mock_payload["mock_detector"]["error_model"]["miss"]["slope_px"] = 0.0
    with pytest.raises(DetectionConfigError, match="invalid mock_detector.error_model"):
        MockDetectorConfig.load(_write(tmp_path / "e.yaml", mock_payload))

    mock_payload = yaml.safe_load(MOCK_CONFIG.read_text(encoding="utf-8"))
    mock_payload["mock_detector"]["seed"] = "not-an-int"
    with pytest.raises(DetectionConfigError, match="seed must be an integer"):
        MockDetectorConfig.load(_write(tmp_path / "f.yaml", mock_payload))

    payload = yaml.safe_load(DETECTION_CONFIG.read_text(encoding="utf-8"))
    payload["overlay"]["max_rate_hz"] = 0.0
    with pytest.raises(DetectionConfigError, match="overlay.max_rate_hz must be positive"):
        DetectionConfig.load(_write(tmp_path / "g.yaml", payload))


def test_targets_need_a_model_and_a_usable_position(tmp_path: Path) -> None:
    with pytest.raises(DetectionConfigError, match="position must be"):
        load_targets(_write(tmp_path / "a.yaml", {
            "targets": [{"id": "t", "class": "car", "position": [1.0], "size": [1, 1, 1]}],
            "models": {"car": "yungu_car"}}), ground_z_m=0.0)
    with pytest.raises(DetectionConfigError, match="no Gazebo model"):
        load_targets(_write(tmp_path / "b.yaml", {
            "targets": [{"id": "t", "class": "ufo", "position": [1.0, 2.0], "size": [1, 1, 1]}]}),
            ground_z_m=0.0)
    with pytest.raises(DetectionConfigError, match="duplicate target ids"):
        load_targets(_write(tmp_path / "c.yaml", {
            "models": {"car": "yungu_car"},
            "targets": [
                {"id": "t", "class": "car", "position": [1.0, 2.0], "size": [1, 1, 1]},
                {"id": "t", "class": "car", "position": [3.0, 4.0], "size": [1, 1, 1]},
            ]}), ground_z_m=0.0)


def test_workspace_paths_resolve_upwards_from_the_configuration() -> None:
    absolute = Path("/tmp/example.stl")
    assert resolve_workspace_path(absolute, reference=DETECTION_CONFIG) == absolute
    # The collision mesh lives in the PX4 submodule, several levels above the
    # configuration directory.
    mesh = resolve_workspace_path(
        "VisionFlow-PX4/Tools/simulation/gz/worlds/yungu_collider.stl",
        reference=DETECTION_CONFIG)
    assert mesh.name == "yungu_collider.stl"
    assert mesh.is_file()
