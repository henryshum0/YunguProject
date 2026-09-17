"""The navigator walks an agent to a goal and tracks where it got to."""

from __future__ import annotations

from math import pi, radians

import pytest

from agents.navigator import AgentLimits, Navigator, Pose2D, wrap_angle

DT = 0.02


def _navigator(pose: Pose2D | None = None, **limits: float) -> Navigator:
    defaults = {"speed_mps": 1.0, "yaw_rate_rps": radians(90.0), "arrive_radius_m": 0.4}
    defaults.update(limits)
    return Navigator(pose or Pose2D(), limits=AgentLimits(**defaults))


def _run(navigator: Navigator, ticks: int) -> float:
    """Run the navigator and return the total ground distance covered."""
    return sum(navigator.update(DT).distance_m for _ in range(ticks))


class TestIdle:
    def test_an_agent_with_no_goal_stands_still(self) -> None:
        step = _navigator().update(DT)
        assert step.command.is_stopped
        assert step.distance_m == 0.0
        assert step.arrived

    def test_cancel_stops_the_agent(self) -> None:
        navigator = _navigator()
        navigator.set_goal(10.0, 0.0)
        navigator.update(DT)
        navigator.cancel()
        step = navigator.update(DT)
        assert step.command.is_stopped
        assert navigator.goal is None


class TestWalking:
    def test_it_walks_straight_at_a_goal_it_already_faces(self) -> None:
        navigator = _navigator()
        navigator.set_goal(5.0, 0.0)
        step = navigator.update(DT)
        assert step.command.linear_x == pytest.approx(1.0)
        assert step.distance_m > 0.0

    def test_it_turns_on_the_spot_before_walking_backwards(self) -> None:
        """A goal behind the agent must not be reached by sliding sideways."""
        navigator = _navigator()
        navigator.set_goal(-5.0, 0.0)
        step = navigator.update(DT)
        assert step.command.linear_x == 0.0
        assert step.command.angular_z != 0.0
        assert step.distance_m == 0.0

    def test_it_turns_then_walks(self) -> None:
        navigator = _navigator(Pose2D(yaw=pi))
        navigator.set_goal(5.0, 0.0)
        # Long enough to rotate 180 degrees at 90 deg/s and then move.
        assert _run(navigator, 200) > 0.0
        assert navigator.pose.x > 0.0

    def test_yaw_rate_is_limited(self) -> None:
        navigator = _navigator(Pose2D(yaw=pi))
        navigator.set_goal(5.0, 0.0)
        step = navigator.update(DT)
        assert abs(step.command.angular_z) <= radians(90.0) + 1e-9


class TestArrival:
    def test_it_stops_inside_the_arrive_radius(self) -> None:
        navigator = _navigator(Pose2D(x=4.8, y=0.0))
        navigator.set_goal(5.0, 0.0)
        step = navigator.update(DT)
        assert step.arrived
        assert step.command.is_stopped
        assert navigator.arrived

    def test_it_reaches_a_goal_and_holds(self) -> None:
        navigator = _navigator()
        navigator.set_goal(3.0, 0.0)
        _run(navigator, 400)
        assert navigator.arrived
        assert navigator.distance_to_goal <= 0.4
        # Having arrived it stays put rather than creeping.
        assert _run(navigator, 50) == 0.0

    def test_it_never_overshoots_within_one_tick(self) -> None:
        """Close to the goal the step is clamped to the remaining distance."""
        navigator = _navigator(Pose2D(x=4.59, y=0.0), arrive_radius_m=0.01)
        navigator.set_goal(5.0, 0.0)
        step = navigator.update(1.0)
        assert step.pose.x <= 5.0 + 1e-9

    def test_a_new_goal_restarts_a_finished_agent(self) -> None:
        navigator = _navigator()
        navigator.set_goal(1.0, 0.0)
        _run(navigator, 200)
        assert navigator.arrived
        navigator.set_goal(-4.0, 0.0)
        assert not navigator.arrived
        assert not navigator.update(DT).arrived


class TestDeadReckoning:
    def test_pose_tracks_the_commanded_velocity(self) -> None:
        """No pose feedback exists, so integrating the command must be right."""
        navigator = _navigator()
        navigator.set_goal(100.0, 0.0)
        covered = _run(navigator, 100)
        assert covered == pytest.approx(1.0 * 100 * DT)
        assert navigator.pose.x == pytest.approx(covered, abs=1e-6)
        assert navigator.pose.y == pytest.approx(0.0, abs=1e-6)

    def test_it_walks_to_an_off_axis_goal(self) -> None:
        navigator = _navigator()
        navigator.set_goal(3.0, 3.0)
        _run(navigator, 600)
        assert navigator.arrived
        assert navigator.pose.x == pytest.approx(3.0, abs=0.5)
        assert navigator.pose.y == pytest.approx(3.0, abs=0.5)


class TestLimits:
    @pytest.mark.parametrize("field", ["speed_mps", "yaw_rate_rps", "arrive_radius_m"])
    def test_non_positive_limits_are_rejected(self, field: str) -> None:
        with pytest.raises(ValueError, match=field):
            AgentLimits(**{field: 0.0})

    def test_dt_must_be_positive(self) -> None:
        with pytest.raises(ValueError, match="dt"):
            _navigator().update(0.0)


@pytest.mark.parametrize(("angle", "expected"), [
    (0.0, 0.0),
    (2 * pi + 0.5, 0.5),
    (-0.5, -0.5),
    (pi + 0.5, -pi + 0.5),
    # Exactly opposite wraps to the -pi end of the range.
    (3 * pi, -pi),
    (-3 * pi, -pi),
])
def test_wrap_angle(angle: float, expected: float) -> None:
    assert wrap_angle(angle) == pytest.approx(expected)
