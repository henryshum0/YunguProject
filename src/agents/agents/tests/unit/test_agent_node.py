"""The driver node's control tick, exercised end to end.

The tick is the one place where the config, the navigator, the gait and the ROS
publishers all meet, and it runs on a timer rather than on a call — so a stale
attribute in it does not fail a build or a config test, it kills the node half a
second after launch and leaves the robots adrift with nothing driving them. That
is exactly what happened, so the tick is smoke-tested here against the shipped
config: build the node, drive it, and require that it survives.
"""

from __future__ import annotations

from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="needs a sourced ROS 2 environment")

from agents.config import AgentsConfig  # noqa: E402
from agents.ros.agent_node import AgentDriverNode  # noqa: E402

SHIPPED_CONFIG = Path(__file__).resolve().parents[3] / "config" / "agents.yaml"


@pytest.fixture
def node():
    rclpy.init()
    driver = AgentDriverNode(AgentsConfig.load(SHIPPED_CONFIG), rate_hz=50.0)
    try:
        yield driver
    finally:
        driver.destroy_node()
        rclpy.shutdown()


def test_an_idle_tick_runs_without_error(node) -> None:
    """Every agent is idle at startup; the tick must still complete."""
    for _ in range(3):
        node._tick()


def test_a_walking_tick_runs_without_error(node) -> None:
    """The moving path touches the navigator, the gait and every publisher."""
    for runtime in node._runtimes:
        runtime.navigator.set_goal(runtime.spec.spawn.x + 10.0, runtime.spec.spawn.y)
    for _ in range(20):
        node._tick()
    assert any(runtime.phase > 0.0 for runtime in node._runtimes), "no agent advanced its gait"


def test_a_goal_message_starts_an_agent_walking(node) -> None:
    from geometry_msgs.msg import PoseStamped

    runtime = node._runtimes[0]
    goal = PoseStamped()
    goal.pose.position.x = runtime.spec.spawn.x + 5.0
    goal.pose.position.y = runtime.spec.spawn.y
    node._on_goal(runtime.spec.name, goal)

    assert runtime.navigator.goal is not None
    node._tick()
    assert not runtime.navigator.arrived


class TestTheClockItIntegratesOn:
    """Dead reckoning has to advance on the clock Gazebo moves the robots with.

    Gazebo applies a commanded velocity over *simulated* time. Integrating a tick
    of wall clock instead credits the agent with motion it never made, by exactly
    the fraction the real-time factor falls short of 1 -- and nothing corrects it,
    so the robot ends up somewhere the map never showed it going.
    """

    @staticmethod
    def _clock(seconds: float):
        from rosgraph_msgs.msg import Clock
        message = Clock()
        message.clock.sec = int(seconds)
        message.clock.nanosec = int(round((seconds - int(seconds)) * 1e9))
        return message

    def test_the_interval_is_measured_between_clock_readings(self, node) -> None:
        node._on_clock(self._clock(100.0))
        assert node._elapsed() is None, "the first clocked tick has no interval yet"
        node._on_clock(self._clock(100.25))
        assert node._elapsed() == pytest.approx(0.25)

    def test_a_slow_simulation_yields_a_shorter_interval_than_the_tick(self, node) -> None:
        """The bug, stated as a test: at RTF 0.8 a 20 ms tick is 16 ms of motion."""
        node._on_clock(self._clock(10.0))
        node._elapsed()
        node._on_clock(self._clock(10.0 + 0.020 * 0.8))
        assert node._elapsed() == pytest.approx(0.016)

    def test_a_paused_simulation_skips_the_tick(self, node) -> None:
        """Nothing moved, so nothing may be integrated -- and the last command
        stays latched in Gazebo, which is what holds the agent still."""
        node._on_clock(self._clock(5.0))
        node._elapsed()
        assert node._elapsed() is None

    def test_a_reset_simulation_is_survived(self, node) -> None:
        node._on_clock(self._clock(80.0))
        node._elapsed()
        node._on_clock(self._clock(0.5))
        assert node._elapsed() is None, "time going backwards must not integrate"
        node._on_clock(self._clock(0.7))
        assert node._elapsed() == pytest.approx(0.2), "and it must recover after"

    def test_no_interval_is_lost_when_ticks_are_skipped(self, node) -> None:
        """A starved node still has to account for every simulated second:
        Gazebo kept applying the latched command throughout."""
        node._on_clock(self._clock(0.0))
        node._elapsed()
        node._on_clock(self._clock(0.4))
        assert node._elapsed() == pytest.approx(0.4)

    def test_without_a_clock_it_falls_back_and_says_so(self, node) -> None:
        assert node._elapsed() == pytest.approx(node._nominal_dt)
        assert node._warned_about_the_clock, "a silent fallback is the bug coming back"

    def test_a_tick_with_no_clock_interval_publishes_nothing(self, node) -> None:
        runtime = node._runtimes[0]
        published: list = []
        runtime.cmd_vel = type("Fake", (), {"publish": lambda _s, m: published.append(m)})()
        node._on_clock(self._clock(3.0))
        node._tick()   # first clocked tick: no interval
        node._tick()   # clock has not moved: paused
        assert published == []


def test_the_reported_pose_is_built_from_the_ground(node) -> None:
    """The ENU origin is the launch point, so pose z must include the floor."""
    runtime = node._runtimes[0]
    published: list = []
    runtime.pose_publisher = type("Fake", (), {"publish": lambda _self, m: published.append(m)})()
    node._tick()

    expected = node._config.ground_z_m + runtime.spec.base_height_m
    assert published[-1].pose.position.z == pytest.approx(expected)
    assert published[-1].header.frame_id == "map"
