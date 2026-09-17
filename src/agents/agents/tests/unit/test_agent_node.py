"""The driver node's control tick, exercised end to end.

The tick is the one place where the config, the navigator, the gait and the ROS
publishers all meet, and it runs on a timer rather than on a call — so a stale
attribute in it does not fail a build or a config test, it kills the node half a
second after launch and leaves the robots adrift with nothing driving them. That
is exactly what happened, so the tick is smoke-tested here against the shipped
config: build the node, drive it, and require that it survives.
"""

from __future__ import annotations

import signal
import sys
import time
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


class TestParkingOnTheWayOut:
    """Gazebo's velocity controller latches, so stopping the driver is not enough.

    Whatever it heard last it keeps applying forever. A driver that dies mid-walk
    therefore leaves its robot walking, and nothing in the simulation stops it
    again -- a G1 ended up 50 m from its spawn that way, long after the driver
    had gone.
    """

    @staticmethod
    def _capture(runtime) -> tuple[list, dict[str, list]]:
        twists: list = []
        runtime.cmd_vel = type("Fake", (), {"publish": lambda _s, m: twists.append(m)})()
        angles: dict[str, list] = {}
        for joint in list(runtime.joint_publishers):
            sink = angles.setdefault(joint, [])
            runtime.joint_publishers[joint] = type(
                "Fake", (), {"publish": lambda _s, m, sink=sink: sink.append(m.data)})()
        return twists, angles

    def test_every_agent_is_commanded_to_a_dead_stop(self, node) -> None:
        captured = [self._capture(runtime) for runtime in node._runtimes]
        for runtime in node._runtimes:
            runtime.navigator.set_goal(runtime.spec.spawn.x + 20.0, runtime.spec.spawn.y)
        node.park(settle_s=0.0)

        for (twists, _), runtime in zip(captured, node._runtimes):
            assert twists, f"{runtime.spec.name} was never told to stop"
            assert twists[-1].linear.x == 0.0
            assert twists[-1].angular.z == 0.0

    def test_the_legs_are_parked_in_the_standing_stance(self, node) -> None:
        """The joint controllers latch too, so the legs freeze mid-stride."""
        captured = [self._capture(runtime) for runtime in node._runtimes]
        node.park(settle_s=0.0)
        for (_, angles), runtime in zip(captured, node._runtimes):
            stance = runtime.gait.stance()
            for joint, sent in angles.items():
                assert sent, f"{joint} was left wherever it happened to be"
                assert sent[-1] == pytest.approx(stance[joint])

    def test_the_goal_is_dropped_so_nothing_resumes_it(self, node) -> None:
        for runtime in node._runtimes:
            runtime.navigator.set_goal(0.0, 0.0)
        node.park(settle_s=0.0)
        for runtime in node._runtimes:
            assert runtime.navigator.goal is None

    def test_the_control_timer_stops_first(self, node) -> None:
        """Otherwise a tick after parking would command the agent onwards again."""
        node.park(settle_s=0.0)
        assert node._timer.is_canceled()


def test_an_interrupted_driver_parks_before_it_exits(tmp_path) -> None:
    """The wiring, not just the method: ``main`` must park on the way out.

    Testing ``park`` alone does not catch the shutdown path forgetting to call
    it, and that path is the whole point -- so this runs the driver as its own
    process and interrupts it. No simulator is needed: the commands going
    nowhere is fine, what matters is that they are sent at all, with the node
    still alive enough to send them. rclpy's own signal handler tears the
    context down before ``spin`` returns, which is exactly the trap here.
    """
    import subprocess

    source_root = Path(__file__).resolve().parents[2] / "src"
    script = (
        "import sys;"
        f"sys.path.insert(0, {str(source_root)!r});"
        "from agents.ros.agent_node import main;"
        f"sys.exit(main(['--config', {str(SHIPPED_CONFIG)!r}, '--settle-s', '0.05']))"
    )
    log = tmp_path / "driver.log"
    with log.open("w") as sink:
        driver = subprocess.Popen(
            [sys.executable, "-c", script], stdout=sink, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 30.0
        while "driving" not in log.read_text(errors="replace"):
            assert driver.poll() is None, f"driver exited early:\n{log.read_text()}"
            assert time.monotonic() < deadline, f"driver never started:\n{log.read_text()}"
            time.sleep(0.2)
        driver.send_signal(signal.SIGINT)
        assert driver.wait(timeout=30) == 0
    finally:
        if driver.poll() is None:
            driver.kill()
    assert "stopping: commanded" in log.read_text(errors="replace"), (
        f"the driver exited without stopping its agents:\n{log.read_text()}")


def test_the_reported_pose_is_built_from_the_ground(node) -> None:
    """The ENU origin is the launch point, so pose z must include the floor."""
    runtime = node._runtimes[0]
    published: list = []
    runtime.pose_publisher = type("Fake", (), {"publish": lambda _self, m: published.append(m)})()
    node._tick()

    expected = node._config.ground_z_m + runtime.spec.base_height_m
    assert published[-1].pose.position.z == pytest.approx(expected)
    assert published[-1].header.frame_id == "map"
