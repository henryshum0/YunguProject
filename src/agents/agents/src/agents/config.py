"""Validated configuration for the scripted ground agents.

One YAML file describes every agent: which model to spawn, where it starts, how
fast it walks, and how its legs are animated. The spawner, the agent node and the
launch file all read this same file, so adding an agent is a config change rather
than a code change — the same property ``src/detection/config/targets.yaml`` has
for search targets.

Positions are ENU metres in the navigation frame, the frame coverage areas and
waypoints already use, so a goal can be taken straight off the operations map.
``world_origin_in_gz`` converts that to Gazebo world coordinates for spawning,
exactly as the detection target spawner does.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import radians
from pathlib import Path

import yaml

from agents.gait import Gait, gait_from_mapping
from agents.navigator import AgentLimits, Pose2D


class AgentConfigError(ValueError):
    """Raised when the agents configuration cannot safely drive an agent."""


@dataclass(frozen=True, slots=True)
class AgentCamera:
    """A forward-facing camera carried by an agent.

    The upstream descriptions' own cameras are stripped during conversion, so
    this is the one deliberate sensor an agent carries: it is what the operator
    GUI shows, giving each robot a view of its own.
    """

    topic: str
    #: Mount position relative to the agent's root link, in metres.
    pose: tuple[float, float, float]
    #: Downward tilt, radians. Positive pitches the view towards the ground.
    pitch_rad: float
    hfov_rad: float
    width: int
    height: int
    update_rate_hz: float

    def __post_init__(self) -> None:
        if not self.topic.startswith("/"):
            raise ValueError("camera.topic must be an absolute ROS/Gazebo name")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        if not 0.0 < self.hfov_rad < 2.0 * 3.14159265:
            raise ValueError("camera hfov_deg must be between 0 and 360")
        if self.update_rate_hz <= 0.0:
            raise ValueError("camera update_rate_hz must be positive")


@dataclass(frozen=True, slots=True)
class AgentSpec:
    """Everything needed to spawn and drive one agent."""

    name: str
    model: str
    spawn: Pose2D
    #: Height of the model origin above the ENU ground plane, in metres.
    ground_z_m: float
    limits: AgentLimits
    gait: Gait
    camera: AgentCamera | None = None

    @property
    def goal_topic(self) -> str:
        """Where an operator sends this agent a goal."""
        return f"/agents/{self.name}/goal_pose"

    @property
    def pose_topic(self) -> str:
        """Where this agent reports the pose it believes it is at."""
        return f"/agents/{self.name}/pose"


@dataclass(frozen=True, slots=True)
class AgentsConfig:
    """The whole agents contract."""

    #: ENU origin expressed in Gazebo world coordinates, for spawning.
    world_origin_in_gz: tuple[float, float, float]
    agents: tuple[AgentSpec, ...]
    config_file: Path

    def agent(self, name: str) -> AgentSpec:
        for spec in self.agents:
            if spec.name == name:
                return spec
        raise AgentConfigError(
            f"no agent named '{name}'; configured: {', '.join(spec.name for spec in self.agents)}")

    def gz_spawn_pose(self, spec: AgentSpec) -> tuple[float, float, float]:
        """``spec``'s spawn position converted to Gazebo world coordinates."""
        origin = self.world_origin_in_gz
        return (spec.spawn.x + origin[0], spec.spawn.y + origin[1], spec.ground_z_m + origin[2])

    @classmethod
    def load(cls, path: str | Path) -> "AgentsConfig":
        """Read and validate the agents YAML."""
        config_file = Path(path).expanduser().resolve()
        if not config_file.is_file():
            raise AgentConfigError(f"agents config file does not exist: '{config_file}'")
        try:
            payload = yaml.safe_load(config_file.read_text(encoding="utf-8"))
        except OSError as exc:
            raise AgentConfigError(f"cannot read agents config '{config_file}': {exc}") from exc
        except yaml.YAMLError as exc:
            raise AgentConfigError(f"invalid YAML in agents config '{config_file}': {exc}") from exc
        if not isinstance(payload, Mapping):
            raise AgentConfigError(f"agents config '{config_file}' must contain a YAML mapping")
        return cls.from_mapping(payload, config_file=config_file)

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object], *, config_file: Path) -> "AgentsConfig":
        origin = _origin(payload.get("world_origin_in_gz"))
        entries = payload.get("agents")
        if not isinstance(entries, Sequence) or isinstance(entries, (str, bytes)) or not entries:
            raise AgentConfigError("agents must be a non-empty list")

        specs: list[AgentSpec] = []
        seen: set[str] = set()
        for index, entry in enumerate(entries):
            spec = _agent(entry, index)
            if spec.name in seen:
                raise AgentConfigError(f"duplicate agent name '{spec.name}'")
            seen.add(spec.name)
            specs.append(spec)
        return cls(world_origin_in_gz=origin, agents=tuple(specs), config_file=config_file)


def _origin(value: object) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise AgentConfigError("world_origin_in_gz must be a list of three numbers")
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError) as exc:
        raise AgentConfigError(f"world_origin_in_gz must be numeric: {exc}") from exc


def _agent(entry: object, index: int) -> AgentSpec:
    where = f"agents[{index}]"
    if not isinstance(entry, Mapping):
        raise AgentConfigError(f"{where} must be a mapping")
    name = str(entry.get("name", "")).strip()
    if not name:
        raise AgentConfigError(f"{where}.name is required")
    model = str(entry.get("model", "")).strip()
    if not model:
        raise AgentConfigError(f"{where}.model is required")

    spawn_payload = entry.get("spawn")
    if not isinstance(spawn_payload, Mapping):
        raise AgentConfigError(f"{where}.spawn is required and must be a mapping")
    try:
        spawn = Pose2D(
            x=float(spawn_payload.get("x", 0.0)),
            y=float(spawn_payload.get("y", 0.0)),
            yaw=radians(float(spawn_payload.get("yaw_deg", 0.0))),
        )
    except (TypeError, ValueError) as exc:
        raise AgentConfigError(f"{where}.spawn is invalid: {exc}") from exc

    gait_payload = entry.get("gait")
    if not isinstance(gait_payload, Mapping):
        raise AgentConfigError(f"{where}.gait is required and must be a mapping")
    try:
        gait = gait_from_mapping(gait_payload)
    except ValueError as exc:
        raise AgentConfigError(f"{where}.{exc}") from exc

    try:
        return AgentSpec(
            name=name,
            model=model,
            spawn=spawn,
            ground_z_m=float(entry.get("ground_z_m", 0.0)),
            limits=_limits(entry.get("limits"), where),
            gait=gait,
            camera=_camera(entry.get("camera"), where, name),
        )
    except ValueError as exc:
        raise AgentConfigError(f"{where} is invalid: {exc}") from exc


def _camera(value: object, where: str, agent_name: str) -> AgentCamera | None:
    """Parse an agent's optional forward camera."""
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise AgentConfigError(f"{where}.camera must be a mapping")
    known = {"topic", "pose", "pitch_deg", "hfov_deg", "width", "height", "update_rate_hz"}
    unknown = set(value) - known
    if unknown:
        raise AgentConfigError(f"{where}.camera has unknown key(s): {', '.join(sorted(unknown))}")
    pose = value.get("pose", [0.0, 0.0, 0.0])
    if not isinstance(pose, Sequence) or isinstance(pose, (str, bytes)) or len(pose) != 3:
        raise AgentConfigError(f"{where}.camera.pose must be a list of three numbers")
    try:
        return AgentCamera(
            topic=str(value.get("topic", f"/agents/{agent_name}/front_camera/image")),
            pose=(float(pose[0]), float(pose[1]), float(pose[2])),
            pitch_rad=radians(float(value.get("pitch_deg", 0.0))),
            hfov_rad=radians(float(value.get("hfov_deg", 90.0))),
            width=int(value.get("width", 640)),
            height=int(value.get("height", 480)),
            update_rate_hz=float(value.get("update_rate_hz", 30.0)),
        )
    except (TypeError, ValueError) as exc:
        raise AgentConfigError(f"{where}.camera is invalid: {exc}") from exc


def _limits(value: object, where: str) -> AgentLimits:
    if value is None:
        return AgentLimits()
    if not isinstance(value, Mapping):
        raise AgentConfigError(f"{where}.limits must be a mapping")
    unknown = set(value) - {"speed_mps", "yaw_rate_dps", "arrive_radius_m", "turn_in_place_deg"}
    if unknown:
        raise AgentConfigError(f"{where}.limits has unknown key(s): {', '.join(sorted(unknown))}")
    defaults = AgentLimits()
    try:
        return AgentLimits(
            speed_mps=float(value.get("speed_mps", defaults.speed_mps)),
            yaw_rate_rps=radians(float(value.get("yaw_rate_dps", 60.0))),
            arrive_radius_m=float(value.get("arrive_radius_m", defaults.arrive_radius_m)),
            turn_in_place_rad=radians(float(value.get("turn_in_place_deg", 35.0))),
        )
    except (TypeError, ValueError) as exc:
        raise AgentConfigError(f"{where}.limits is invalid: {exc}") from exc
