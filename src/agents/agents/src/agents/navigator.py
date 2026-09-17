"""Kinematic go-to-goal for a scripted ground agent.

Given a goal, this produces the body-frame velocity that walks an agent to it,
and integrates that velocity to keep its own idea of where the agent is. There
is no state estimator and no pose feedback on purpose: the agent has no
collisions and no gravity, so the simulator applies exactly the velocity it is
given. Commanded motion *is* actual motion, and dead reckoning from the spawn
pose stays correct. Closing a loop around a pose topic here would add latency and
a failure mode to a system that cannot drift.

The control law is deliberately the simple one an operator can predict: turn on
the spot until roughly facing the goal, then walk, steering out the remaining
heading error as it goes. It stops inside ``arrive_radius_m`` rather than
converging asymptotically, so an agent parks and holds a clean stance instead of
creeping.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, cos, degrees, hypot, pi, radians, sin


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to ``[-pi, pi)``.

    Exactly-opposite headings land on ``-pi``. Which side of the turn that picks
    is arbitrary and either is correct: the agent is being asked to turn around.
    """
    return (angle_rad + pi) % (2.0 * pi) - pi


@dataclass(frozen=True, slots=True)
class Pose2D:
    """An agent pose in the navigation ENU frame."""

    x: float = 0.0
    y: float = 0.0
    #: ENU yaw in radians: 0 faces east, positive counter-clockwise.
    yaw: float = 0.0

    @property
    def yaw_deg(self) -> float:
        return degrees(self.yaw)


@dataclass(frozen=True, slots=True)
class Command:
    """A body-frame velocity command: forward speed and yaw rate."""

    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def is_stopped(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


@dataclass(frozen=True, slots=True)
class AgentLimits:
    """Motion limits for one agent."""

    speed_mps: float = 0.8
    yaw_rate_rps: float = radians(60.0)
    #: The agent stops once the goal is this close.
    arrive_radius_m: float = 0.4
    #: Above this heading error the agent turns on the spot instead of walking.
    turn_in_place_rad: float = radians(35.0)

    def __post_init__(self) -> None:
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        if self.yaw_rate_rps <= 0.0:
            raise ValueError("yaw_rate_rps must be positive")
        if self.arrive_radius_m <= 0.0:
            raise ValueError("arrive_radius_m must be positive")


@dataclass(frozen=True, slots=True)
class Step:
    """What one control tick produced."""

    command: Command
    pose: Pose2D
    #: Ground distance covered this tick, which is what advances the gait.
    distance_m: float
    arrived: bool


class Navigator:
    """Walks one agent to successive goals, tracking its pose by dead reckoning."""

    def __init__(self, pose: Pose2D, *, limits: AgentLimits | None = None) -> None:
        self._pose = pose
        self._limits = limits or AgentLimits()
        self._goal: tuple[float, float] | None = None
        self._arrived = True

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def goal(self) -> tuple[float, float] | None:
        """The active goal, or ``None`` when the agent is idle."""
        return self._goal

    @property
    def arrived(self) -> bool:
        """Whether the agent is holding position rather than walking."""
        return self._arrived

    @property
    def distance_to_goal(self) -> float | None:
        if self._goal is None:
            return None
        return hypot(self._goal[0] - self._pose.x, self._goal[1] - self._pose.y)

    def set_goal(self, x: float, y: float) -> None:
        """Send the agent to an ENU position. Replaces any goal in progress."""
        self._goal = (float(x), float(y))
        self._arrived = False

    def cancel(self) -> None:
        """Stop where it stands and forget the goal."""
        self._goal = None
        self._arrived = True

    def update(self, dt: float) -> Step:
        """Advance one control tick and return the command to publish."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if self._goal is None or self._arrived:
            return Step(Command(), self._pose, 0.0, arrived=True)

        to_goal_x = self._goal[0] - self._pose.x
        to_goal_y = self._goal[1] - self._pose.y
        distance = hypot(to_goal_x, to_goal_y)
        if distance <= self._limits.arrive_radius_m:
            self._arrived = True
            return Step(Command(), self._pose, 0.0, arrived=True)

        heading_error = wrap_angle(atan2(to_goal_y, to_goal_x) - self._pose.yaw)
        yaw_rate = _clamp(heading_error / dt, self._limits.yaw_rate_rps)

        if abs(heading_error) > self._limits.turn_in_place_rad:
            # Too far off to walk usefully: rotate on the spot first.
            speed = 0.0
        else:
            # Never overshoot the goal within one tick.
            speed = min(self._limits.speed_mps, distance / dt)

        moved = speed * dt
        yaw = wrap_angle(self._pose.yaw + yaw_rate * dt)
        self._pose = replace(
            self._pose,
            x=self._pose.x + moved * cos(yaw),
            y=self._pose.y + moved * sin(yaw),
            yaw=yaw,
        )
        return Step(Command(speed, yaw_rate), self._pose, moved, arrived=False)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
