"""The agents config is the single source of truth; these keep it honest."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from agents.config import AgentConfigError, AgentsConfig

#: The shipped config, two directories up from tests/unit.
SHIPPED_CONFIG = Path(__file__).resolve().parents[3] / "config" / "agents.yaml"


def _payload(**overrides: object) -> dict:
    payload = {
        "world_origin_in_gz": [0.0, 0.0, 1.0],
        "agents": [{
            "name": "go1",
            "model": "unitree_go1",
            "spawn": {"x": 1.0, "y": 2.0, "yaw_deg": 90.0},
            "gait": {"stride_m": 0.4, "joints": {"FR_thigh_joint": {"amplitude": 0.3}}},
        }],
    }
    payload.update(overrides)
    return payload


def _config(**overrides: object) -> AgentsConfig:
    return AgentsConfig.from_mapping(_payload(**overrides), config_file=Path("/tmp/agents.yaml"))


class TestShippedConfig:
    """The file the launch actually loads must stay valid and consistent."""

    def test_it_loads(self) -> None:
        config = AgentsConfig.load(SHIPPED_CONFIG)
        assert {spec.name for spec in config.agents} == {"go1", "g1"}

    def test_the_quadruped_trots_in_diagonal_pairs(self) -> None:
        """FR+RL must share a phase and be opposite FL+RR, or it is not a trot."""
        gait = AgentsConfig.load(SHIPPED_CONFIG).agent("go1").gait
        offsets = {name: joint.offset for name, joint in gait.joints.items()}
        assert offsets["FR_thigh_joint"] == offsets["RL_thigh_joint"]
        assert offsets["FL_thigh_joint"] == offsets["RR_thigh_joint"]
        assert abs(offsets["FR_thigh_joint"] - offsets["FL_thigh_joint"]) == pytest.approx(0.5)

    def test_the_humanoid_legs_are_half_a_cycle_apart(self) -> None:
        gait = AgentsConfig.load(SHIPPED_CONFIG).agent("g1").gait
        left = gait.joints["left_hip_pitch_joint"].offset
        right = gait.joints["right_hip_pitch_joint"].offset
        assert abs(left - right) == pytest.approx(0.5)

    def test_joint_angles_stay_inside_the_hardware_limits(self) -> None:
        """Bias +/- amplitude must not command a joint past its URDF limit.

        These are the limits the Unitree descriptions declare; commanding past
        them is what makes a leg visibly snap or invert.
        """
        limits = {
            "FR_thigh_joint": (-0.686, 4.501), "FL_thigh_joint": (-0.686, 4.501),
            "RR_thigh_joint": (-0.686, 4.501), "RL_thigh_joint": (-0.686, 4.501),
            "FR_calf_joint": (-2.818, -0.888), "FL_calf_joint": (-2.818, -0.888),
            "RR_calf_joint": (-2.818, -0.888), "RL_calf_joint": (-2.818, -0.888),
            "left_knee_joint": (-0.087, 2.880), "right_knee_joint": (-0.087, 2.880),
            "left_ankle_pitch_joint": (-0.873, 0.524),
            "right_ankle_pitch_joint": (-0.873, 0.524),
            "left_hip_pitch_joint": (-2.531, 2.880), "right_hip_pitch_joint": (-2.531, 2.880),
            "left_shoulder_pitch_joint": (-3.089, 2.670),
            "right_shoulder_pitch_joint": (-3.089, 2.670),
        }
        config = AgentsConfig.load(SHIPPED_CONFIG)
        for spec in config.agents:
            for name, joint in spec.gait.joints.items():
                assert name in limits, f"{name} has no limit recorded in this test"
                low, high = limits[name]
                samples = [joint.angle(index / 64.0) for index in range(64)]
                assert min(samples) >= low - 1e-6, f"{name} goes below its lower limit"
                assert max(samples) <= high + 1e-6, f"{name} goes above its upper limit"


class TestSpawnConversion:
    def test_enu_spawn_becomes_gazebo_world_coordinates(self) -> None:
        config = _config(world_origin_in_gz=[10.0, 20.0, 1.5])
        spec = config.agent("go1")
        assert config.gz_spawn_pose(spec) == pytest.approx((11.0, 22.0, 1.5))

    def test_ground_height_offsets_the_spawn(self) -> None:
        payload = _payload()
        payload["agents"][0]["ground_z_m"] = 0.32
        config = AgentsConfig.from_mapping(payload, config_file=Path("/tmp/a.yaml"))
        assert config.gz_spawn_pose(config.agent("go1"))[2] == pytest.approx(1.32)


class TestTopics:
    def test_each_agent_gets_its_own_topics(self) -> None:
        spec = _config().agent("go1")
        assert spec.goal_topic == "/agents/go1/goal_pose"
        assert spec.pose_topic == "/agents/go1/pose"


class TestValidation:
    def test_unknown_agent_is_reported(self) -> None:
        with pytest.raises(AgentConfigError, match="no agent named 'nope'"):
            _config().agent("nope")

    def test_duplicate_names_are_rejected(self) -> None:
        payload = _payload()
        payload["agents"].append(dict(payload["agents"][0]))
        with pytest.raises(AgentConfigError, match="duplicate agent name"):
            AgentsConfig.from_mapping(payload, config_file=Path("/tmp/a.yaml"))

    def test_an_empty_agent_list_is_rejected(self) -> None:
        with pytest.raises(AgentConfigError, match="non-empty list"):
            _config(agents=[])

    def test_a_missing_gait_is_rejected(self) -> None:
        payload = _payload()
        del payload["agents"][0]["gait"]
        with pytest.raises(AgentConfigError, match="gait"):
            AgentsConfig.from_mapping(payload, config_file=Path("/tmp/a.yaml"))

    def test_a_bad_gait_names_the_agent(self) -> None:
        payload = _payload()
        payload["agents"][0]["gait"]["stride_m"] = -1.0
        with pytest.raises(AgentConfigError, match=r"agents\[0\]"):
            AgentsConfig.from_mapping(payload, config_file=Path("/tmp/a.yaml"))

    def test_an_unknown_limit_key_is_rejected(self) -> None:
        payload = _payload()
        payload["agents"][0]["limits"] = {"speed_mp": 2.0}
        with pytest.raises(AgentConfigError, match="unknown key"):
            AgentsConfig.from_mapping(payload, config_file=Path("/tmp/a.yaml"))

    def test_a_bad_origin_is_rejected(self) -> None:
        with pytest.raises(AgentConfigError, match="world_origin_in_gz"):
            _config(world_origin_in_gz=[0.0, 0.0])

    def test_a_missing_file_is_reported(self, tmp_path: Path) -> None:
        with pytest.raises(AgentConfigError, match="does not exist"):
            AgentsConfig.load(tmp_path / "nope.yaml")

    def test_invalid_yaml_is_reported(self, tmp_path: Path) -> None:
        bad = tmp_path / "agents.yaml"
        bad.write_text("agents: [oops\n", encoding="utf-8")
        with pytest.raises(AgentConfigError, match="invalid YAML"):
            AgentsConfig.load(bad)

    def test_a_non_mapping_document_is_reported(self, tmp_path: Path) -> None:
        bad = tmp_path / "agents.yaml"
        bad.write_text(yaml.safe_dump([1, 2, 3]), encoding="utf-8")
        with pytest.raises(AgentConfigError, match="mapping"):
            AgentsConfig.load(bad)
