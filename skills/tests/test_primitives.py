from __future__ import annotations

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from coverage_planner.action import PlanCoverage
from offboard_fsm.srv import ClearWaypoints, QueueWaypoints
from skills import (
    ClearWaypointsPrimitive,
    MovePrimitive,
    PlanSearchPrimitive,
    LandPrimitive,
    SkillExecutionError,
    SkillTimeoutError,
    TakeoffPrimitive,
)
from skills.tests.config_data import TEST_CONFIG


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


class FakePlanGoalHandle:
    def __init__(self, response, *, accepted: bool = True, done: bool = True) -> None:
        self.accepted = accepted
        self._result_future = FakeFuture(SimpleNamespace(result=response), done=done)

    def get_result_async(self):
        return self._result_future


class FakePlanActionClient:
    def __init__(
        self,
        response=None,
        *,
        available: bool = True,
        receipt_done: bool = True,
        result_done: bool = True,
        accepted: bool = True,
    ) -> None:
        self.response = response
        self.available = available
        self.receipt_done = receipt_done
        self.result_done = result_done
        self.accepted = accepted
        self.goals = []
        self.receipt_future = None

    def wait_for_server(self, timeout_sec=None) -> bool:
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        goal_handle = FakePlanGoalHandle(
            self.response, accepted=self.accepted, done=self.result_done)
        self.receipt_future = FakeFuture(goal_handle, done=self.receipt_done)
        return self.receipt_future


def _plan_primitive(monkeypatch, client: FakePlanActionClient) -> PlanSearchPrimitive:
    monkeypatch.setattr(
        "skills.primitives.plan_search.ActionClient",
        lambda node, action_type, action_name: client,
    )
    return PlanSearchPrimitive(FakeNode(), config=TEST_CONFIG)


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


def _trigger_response(message: str = "accepted", *, success: bool = True) -> Trigger.Response:
    response = Trigger.Response()
    response.success = success
    response.message = message
    return response


def test_move_queues_every_pose_with_waypoint_buffer_service(monkeypatch) -> None:
    client = FakeClient(_queue_response(2))
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.move.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    goals = (PoseStamped(), PoseStamped())
    primitive = MovePrimitive(node, config=TEST_CONFIG)
    assert primitive.name == "move"
    assert primitive.call(goals) == 2
    assert primitive.queue_service == "/waypoint_buffer"
    assert node.service_name == "/waypoint_buffer"
    assert client.requests[0].waypoints == list(goals)


def test_move_reports_unavailable_or_rejected_queue_service(monkeypatch) -> None:
    unavailable = MovePrimitive(FakeNode(FakeClient(available=False)), config=TEST_CONFIG)
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        unavailable.call((PoseStamped(),))

    client = FakeClient(_queue_response(0, success=False))
    monkeypatch.setattr(
        "skills.primitives.move.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    rejected = MovePrimitive(FakeNode(client), config=TEST_CONFIG)
    with pytest.raises(SkillExecutionError, match="rejected"):
        rejected.call((PoseStamped(),))


def test_clear_waypoints_returns_removed_count_and_times_out(monkeypatch) -> None:
    client = FakeClient(_clear_response(3))
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.clear_waypoints.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = ClearWaypointsPrimitive(node, config=TEST_CONFIG)
    assert primitive.name == "clear_waypoints"
    assert primitive.call() == 3
    assert primitive.clear_service == "/waypoint_buffer/clear"
    assert node.service_name == "/waypoint_buffer/clear"

    timed_out_client = FakeClient(_clear_response(0), done=False)
    timed_out = ClearWaypointsPrimitive(FakeNode(timed_out_client), config=TEST_CONFIG)
    with pytest.raises(SkillTimeoutError, match="did not respond"):
        timed_out.call()
    assert timed_out_client.future.cancelled


@pytest.mark.parametrize(
    ("primitive_type", "expected_name", "expected_service"),
    [
        (TakeoffPrimitive, "takeoff", "/offboard/takeoff"),
        (LandPrimitive, "land", "/offboard/land"),
    ],
)
def test_flight_primitives_call_configured_trigger_services(
    monkeypatch, primitive_type, expected_name, expected_service,
) -> None:
    client = FakeClient(_trigger_response("accepted"))
    node = FakeNode(client)
    monkeypatch.setattr(
        "skills.primitives.flight.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = primitive_type(node, config=TEST_CONFIG)
    assert primitive.name == expected_name
    assert primitive.service_name == expected_service
    assert primitive.call() == "accepted"
    assert node.service_name == expected_service
    assert isinstance(client.requests[0], Trigger.Request)


def test_flight_primitives_report_unavailable_rejected_and_timed_out_services(monkeypatch) -> None:
    unavailable = TakeoffPrimitive(FakeNode(FakeClient(available=False)), config=TEST_CONFIG)
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        unavailable.call()

    rejected_client = FakeClient(_trigger_response("not ready", success=False))
    monkeypatch.setattr(
        "skills.primitives.flight.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    rejected = LandPrimitive(FakeNode(rejected_client), config=TEST_CONFIG)
    with pytest.raises(SkillExecutionError, match="not ready"):
        rejected.call()

    timed_out_client = FakeClient(_trigger_response(), done=False)
    timed_out = TakeoffPrimitive(FakeNode(timed_out_client), config=TEST_CONFIG)
    with pytest.raises(SkillTimeoutError, match="did not respond"):
        timed_out.call()
    assert timed_out_client.future.cancelled


def test_plan_search_returns_coverage_action_waypoints(monkeypatch) -> None:
    response = PlanCoverage.Result()
    response.success = True
    response.waypoints = Path()
    client = FakePlanActionClient(response)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = _plan_primitive(monkeypatch, client)
    result = primitive.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))
    assert result is response.waypoints
    assert client.goals[0].search_area.header.frame_id == "map"
    assert len(client.goals[0].search_area.polygon.points) == 4
    assert client.goals[0].publish_result is False


def test_plan_search_can_request_published_visualization(monkeypatch) -> None:
    response = PlanCoverage.Result()
    response.success = True
    client = FakePlanActionClient(response)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = _plan_primitive(monkeypatch, client)
    primitive.call(
        ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)),
        publish_result=True,
    )
    assert client.goals[0].publish_result is True


def test_search_reports_backend_rejection(monkeypatch) -> None:
    response = PlanCoverage.Result()
    response.success = False
    response.message = "coverage is infeasible"
    client = FakePlanActionClient(response)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    primitive = _plan_primitive(monkeypatch, client)
    with pytest.raises(SkillExecutionError, match="infeasible"):
        primitive.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))


def test_search_reports_unavailable_or_timed_out_service(monkeypatch) -> None:
    unavailable = _plan_primitive(monkeypatch, FakePlanActionClient(available=False))
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        unavailable.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))

    client = FakePlanActionClient(PlanCoverage.Result(), receipt_done=False)
    monkeypatch.setattr(
        "skills.primitives.plan_search.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    timed_out = _plan_primitive(monkeypatch, client)
    with pytest.raises(SkillTimeoutError, match="did not acknowledge"):
        timed_out.call(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))
    assert client.receipt_future.cancelled


def test_search_requires_four_distinct_finite_corners() -> None:
    # Validation occurs before a ROS action is submitted.
    primitive = PlanSearchPrimitive.__new__(PlanSearchPrimitive)
    with pytest.raises(ValueError, match="exactly four"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)))
    with pytest.raises(ValueError, match="distinct"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)))
    with pytest.raises(ValueError, match="finite"):
        primitive.call(((0.0, 0.0), (1.0, 0.0), (1.0, float("nan")), (0.0, 1.0)))
