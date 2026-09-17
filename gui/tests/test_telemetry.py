from __future__ import annotations

from types import SimpleNamespace

import pytest

from gui.telemetry import (
    agent_state_from_pose,
    queue_state_from_path,
    vehicle_state_from_odometry,
)


def _odometry(*, x=3.0, y=-2.0, z=5.0, qz=0.0, qw=1.0):
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw),
    )))


def _path(*points: tuple[float, float]):
    return SimpleNamespace(poses=[SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y))) for x, y in points])


def test_vehicle_odometry_is_converted_to_enu_pose_and_heading() -> None:
    state = vehicle_state_from_odometry(_odometry(qz=2 ** -0.5, qw=2 ** -0.5), received_at=12.0)
    assert (state.x, state.y, state.z, state.heading_deg, state.received_at) == pytest.approx(
        (3.0, -2.0, 5.0, 90.0, 12.0))


def _agent_pose(*, x=5.0, y=-3.0, qz=0.0, qw=1.0):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw),
    ))


def test_agent_pose_is_converted_to_enu_position_and_heading() -> None:
    state = agent_state_from_pose("go1", _agent_pose(qz=2 ** -0.5, qw=2 ** -0.5), received_at=7.0)
    assert state.name == "go1"
    assert (state.x, state.y, state.heading_deg, state.received_at) == pytest.approx(
        (5.0, -3.0, 90.0, 7.0))


def test_agent_pose_rejects_non_finite_values() -> None:
    """A bad pose must not silently park a robot at NaN on the map."""
    with pytest.raises(ValueError, match="go1"):
        agent_state_from_pose("go1", _agent_pose(y=float("inf")))


def test_vehicle_odometry_rejects_non_finite_pose() -> None:
    with pytest.raises(ValueError, match="non-finite"):
        vehicle_state_from_odometry(_odometry(x=float("nan")))


def test_queue_path_preserves_active_then_pending_order_and_empty_state() -> None:
    state = queue_state_from_path(_path((1.0, 2.0), (3.0, 4.0), (5.0, 6.0)), received_at=4.0)
    assert state.points == ((1.0, 2.0), (3.0, 4.0), (5.0, 6.0))
    assert state.received_at == 4.0
    assert queue_state_from_path(_path(), received_at=5.0).points == ()


def test_queue_path_rejects_non_finite_waypoints() -> None:
    with pytest.raises(ValueError, match="waypoint 1"):
        queue_state_from_path(_path((float("inf"), 0.0)))
