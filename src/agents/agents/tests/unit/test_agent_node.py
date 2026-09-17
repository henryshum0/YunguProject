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


def test_the_reported_pose_is_built_from_the_ground(node) -> None:
    """The ENU origin is the launch point, so pose z must include the floor."""
    runtime = node._runtimes[0]
    published: list = []
    runtime.pose_publisher = type("Fake", (), {"publish": lambda _self, m: published.append(m)})()
    node._tick()

    expected = node._config.ground_z_m + runtime.spec.base_height_m
    assert published[-1].pose.position.z == pytest.approx(expected)
    assert published[-1].header.frame_id == "map"
