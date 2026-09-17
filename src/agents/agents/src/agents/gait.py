"""Leg animation: turn distance travelled into joint angles.

The agents do not walk — something moves their base and this makes the legs look
like the reason. The one property that has to hold for that illusion is that the
legs are driven by *distance*, not by wall-clock time: a cycle is completed every
``stride_m`` metres, so the step rate rises and falls with speed and the legs
stop dead when the body does. Animating on a timer instead is what produces the
classic ice-skating look, where the feet scuff along at a fixed cadence while the
body slides at some unrelated speed.

This is the same relationship the UAV already has between its rotor joints and
its motor commands: the joint moves because the vehicle is being commanded to
move, not on a clock of its own.

Each joint is one sinusoid over that cycle:

    angle = bias + amplitude * shape(2*pi * (phase + offset))

``bias`` is the joint's neutral pose, so a robot standing still holds a sensible
stance rather than collapsing to all-zeros. ``offset`` is what makes a gait a
gait: a quadruped trot puts the diagonal pairs half a cycle apart, a biped walk
puts the two legs half a cycle apart. ``shape`` is plain sine for joints that
swing both ways (hips), and a rectified sine for knees, which only bend one way.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import pi, sin

#: Joint angle shapes. ``flex`` is a rectified sine: it stays at zero for half
#: the cycle and bends for the other half, which is how a knee behaves.
SHAPES = ("sin", "flex")


def _shape_value(shape: str, angle_rad: float) -> float:
    if shape == "sin":
        return sin(angle_rad)
    if shape == "flex":
        return max(0.0, sin(angle_rad))
    raise ValueError(f"unknown gait shape '{shape}'; use one of {', '.join(SHAPES)}")


@dataclass(frozen=True, slots=True)
class JointGait:
    """How one joint moves over a gait cycle."""

    #: Neutral angle held when the agent is standing still, in radians.
    bias: float = 0.0
    #: Peak deviation from ``bias``, in radians.
    amplitude: float = 0.0
    #: Position in the cycle, as a fraction of it. 0.5 is the opposite leg.
    offset: float = 0.0
    shape: str = "sin"

    def __post_init__(self) -> None:
        if self.shape not in SHAPES:
            raise ValueError(f"unknown gait shape '{self.shape}'; use one of {', '.join(SHAPES)}")

    def angle(self, phase: float) -> float:
        """Joint angle at ``phase``, where phase is a fraction of the cycle."""
        return self.bias + self.amplitude * _shape_value(self.shape, 2.0 * pi * (phase + self.offset))


@dataclass(frozen=True, slots=True)
class Gait:
    """A whole-robot gait: one cycle per ``stride_m`` metres travelled."""

    #: Ground distance that completes one full cycle, in metres.
    stride_m: float
    joints: Mapping[str, JointGait]

    def __post_init__(self) -> None:
        if self.stride_m <= 0.0:
            raise ValueError("stride_m must be positive")
        if not self.joints:
            raise ValueError("a gait needs at least one joint")

    def advance(self, phase: float, distance_m: float) -> float:
        """Advance ``phase`` by a travelled distance, wrapped to ``[0, 1)``.

        Distance is taken as an absolute value so walking backwards still cycles
        the legs rather than unwinding the phase.
        """
        return (phase + abs(distance_m) / self.stride_m) % 1.0

    def positions(self, phase: float) -> dict[str, float]:
        """Every animated joint's angle at ``phase``."""
        return {name: joint.angle(phase) for name, joint in self.joints.items()}

    def stance(self) -> dict[str, float]:
        """The standing pose: every joint at its bias.

        Used when an agent is idle, so a stopped robot holds its stance instead
        of freezing mid-step at whatever phase it happened to stop on.
        """
        return {name: joint.bias for name, joint in self.joints.items()}

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(self.joints)


def gait_from_mapping(payload: Mapping[str, object]) -> Gait:
    """Build a :class:`Gait` from the plain mapping a YAML config parses to."""
    stride = payload.get("stride_m")
    if not isinstance(stride, (int, float)):
        raise ValueError("gait.stride_m is required and must be a number")
    joints_payload = payload.get("joints")
    if not isinstance(joints_payload, Mapping) or not joints_payload:
        raise ValueError("gait.joints is required and must be a non-empty mapping")

    joints: dict[str, JointGait] = {}
    for name, spec in joints_payload.items():
        if not isinstance(spec, Mapping):
            raise ValueError(f"gait.joints.{name} must be a mapping")
        unknown = set(spec) - {"bias", "amplitude", "offset", "shape"}
        if unknown:
            raise ValueError(
                f"gait.joints.{name} has unknown key(s): {', '.join(sorted(unknown))}")
        try:
            joints[str(name)] = JointGait(
                bias=float(spec.get("bias", 0.0)),
                amplitude=float(spec.get("amplitude", 0.0)),
                offset=float(spec.get("offset", 0.0)),
                shape=str(spec.get("shape", "sin")),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"gait.joints.{name} is invalid: {exc}") from exc
    return Gait(stride_m=float(stride), joints=joints)
