from __future__ import annotations

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from coverage_planner.srv import PlanCoverage
from offboard_fsm.srv import ClearWaypoints, QueueWaypoints
from skills import (
    ClearWaypointsPrimitive,
    MovePrimitive,
    PlanSearchPrimitive,
    SkillExecutionError,
    SkillTimeoutError,
)


class FakeFuture:
    def __init__(self, response, *, done: bool = True) -> None:
        self._response = response
        self._done = done
        self.cancelled = False

    def done(self) -> bool:
        return self._done

    def result(self):
        return self._response

    def cancel(self) -> None:
        self.cancelled = True


class FakeClient:
    def __init__(self, response=None, *, available: bool = True, done: bool = True) -> None:
        self.response = response
        self.available = available
        self.done = done
        self.requests = []
        self.future = None

    def wait_for_service(self, timeout_sec=None) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        self.future = FakeFuture(self.response, done=self.done)
        return self.future


class FakeNode:
    def __init__(self, client=None) -> None:
        self.client = client
        self.service_name = None

    def create_client(self, service_type, service_name):
        self.service_name = service_name
        return self.client


def _queue_response(count: int, *, success: bool = True) -> QueueWaypoints.Response:
    response = QueueWaypoints.Response()
    response.success = success
    response.queued_count = count
    response.message = "queue rejected" if not success else "queued"
    return response


def _clear_response(count: int, *, success: bool = True) -> ClearWaypoints.Response:
    response = ClearWaypoints.Response()
    response.success = success
    response.cleared_count = count
    response.message = "clear rejected" if not success else "cleared"
    return response


def test_move_queues_every_pose_with_waypoint_buffer_service(monkeypatch) -> None:
    client = FakeClient(_queue_response(2))
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.move.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    goals = (PoseStamped(), PoseStamped())
    primitive = MovePrimitive(node)
    assert primitive.name == "move"
    assert primitive.call(goals) == 2
    assert primitive.queue_service == "/waypoint_buffer"
    assert node.service_name == "/waypoint_buffer"
    assert client.requests[0].waypoints == list(goals)


def test_move_reports_unavailable_or_rejected_queue_service(monkeypatch) -> None:
    unavailable = MovePrimitive(FakeNode(FakeClient(available=False)))
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        unavailable.call((PoseStamped(),))

    client = FakeClient(_queue_response(0, success=False))
    monkeypatch.setattr(
        "skills.primitives.move.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    rejected = MovePrimitive(FakeNode(client))
    with pytest.raises(SkillExecutionError, match="rejected"):
        rejected.call((PoseStamped(),))


def test_clear_waypoints_returns_removed_count_and_times_out(monkeypatch) -> None:
    client = FakeClient(_clear_response(3))
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.clear_waypoints.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = ClearWaypointsPrimitive(node)
    assert primitive.name == "clear_waypoints"
    assert primitive.call() == 3
    assert primitive.clear_service == "/waypoint_buffer/clear"
    assert node.service_name == "/waypoint_buffer/clear"

    timed_out_client = FakeClient(_clear_response(0), done=False)
    timed_out = ClearWaypointsPrimitive(FakeNode(timed_out_client))
    with pytest.raises(SkillTimeoutError, match="did not respond"):
        timed_out.call()
    assert timed_out_client.future.cancelled


def test_plan_search_returns_coverage_service_waypoints(monkeypatch) -> None:
    response = PlanCoverage.Response()
    response.success = True
    response.waypoints = Path()
    client = FakeClient(response)
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = PlanSearchPrimitive(node)
    result = primitive.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))
    assert result is response.waypoints
    assert node.service_name == "/coverage_planner/plan_coverage"
    assert client.requests[0].search_area.header.frame_id == "map"
    assert len(client.requests[0].search_area.polygon.points) == 4
    assert client.requests[0].publish_result is False


def test_plan_search_can_request_published_visualization(monkeypatch) -> None:
    response = PlanCoverage.Response()
    response.success = True
    client = FakeClient(response)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = PlanSearchPrimitive(FakeNode(client))
    primitive.call(
        ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)),
        publish_result=True,
    )
    assert client.requests[0].publish_result is True


def test_search_reports_backend_rejection(monkeypatch) -> None:
    response = PlanCoverage.Response()
    response.success = False
    response.message = "coverage is infeasible"
    client = FakeClient(response)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = PlanSearchPrimitive(FakeNode(client))
    with pytest.raises(SkillExecutionError, match="infeasible"):
        primitive.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))


def test_search_reports_unavailable_or_timed_out_service(monkeypatch) -> None:
    unavailable = PlanSearchPrimitive(FakeNode(FakeClient(available=False)))
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        unavailable.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))

    client = FakeClient(PlanCoverage.Response(), done=False)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    timed_out = PlanSearchPrimitive(FakeNode(client))
    with pytest.raises(SkillTimeoutError, match="did not respond"):
        timed_out.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))
    assert client.future.cancelled


def test_search_requires_four_distinct_finite_corners() -> None:
    primitive = PlanSearchPrimitive(FakeNode(FakeClient()))
    with pytest.raises(ValueError, match="exactly four"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)))
    with pytest.raises(ValueError, match="distinct"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)))
    with pytest.raises(ValueError, match="finite"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, float("nan")), (0.0, 1.0)))
