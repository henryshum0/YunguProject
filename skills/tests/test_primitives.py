from __future__ import annotations

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from coverage_planner.srv import PlanCoverage
from skills import (
    MovePrimitive,
    PlanSearchPrimitive,
    SkillExecutionError,
    SkillTimeoutError,
)


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


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
        self.publisher = FakePublisher()
        self.publisher_topic = None
        self.service_name = None

    def create_publisher(self, message_type, topic, qos):
        self.publisher_topic = topic
        return self.publisher

    def create_client(self, service_type, service_name):
        self.service_name = service_name
        return self.client


def test_move_publishes_every_pose_to_reliable_waypoint_buffer() -> None:
    node = FakeNode()
    primitive = MovePrimitive(node)
    goals = (PoseStamped(), PoseStamped())
    assert primitive.name == "move"
    assert primitive.call(goals) == 2
    assert node.publisher_topic == "/waypoint_buffer"
    assert node.publisher.messages == list(goals)


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
