from __future__ import annotations

from math import sqrt

import pytest

from offboard_fsm.srv import ClearWaypoints, QueueWaypoints
from skills import NavigateSkill, SkillTimeoutError
from skills.frames import enu_yaw_quaternion, pose_stamped_from_enu_waypoint


class FakeFuture:
    def __init__(self, response) -> None:
        self._response = response

    def done(self) -> bool:
        return True

    def result(self):
        return self._response


class FakeQueueClient:
    def __init__(self, *, available: bool = True) -> None:
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=None) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        response = QueueWaypoints.Response()
        response.success = True
        response.queued_count = len(request.waypoints)
        return FakeFuture(response)


class FakeClearClient:
    def wait_for_service(self, timeout_sec=None) -> bool:
        return True

    def call_async(self, request):
        response = ClearWaypoints.Response()
        response.success = True
        response.cleared_count = 2
        return FakeFuture(response)


class FakeNode:
    def __init__(self, *, queue_available: bool = True) -> None:
        self.queue_client = FakeQueueClient(available=queue_available)
        self.clear_client = FakeClearClient()
        self.service_names = []

    def create_client(self, service_type, service_name):
        self.service_names.append(service_name)
        if service_type is QueueWaypoints:
            return self.queue_client
        return self.clear_client


def _patch_queue_spin(monkeypatch) -> None:
    monkeypatch.setattr(
        "skills.primitives.move.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )


def test_navigate_converts_single_enu_waypoint_and_queues(monkeypatch) -> None:
    _patch_queue_spin(monkeypatch)
    node = FakeNode()
    navigate = NavigateSkill(node, frame_id="world")
    assert navigate.name == "navigate"
    assert navigate.call((1.0, 2.0, 3.0, 90.0)) == 1
    assert node.service_names == ["/waypoint_buffer", "/waypoint_buffer/clear"]
    pose = node.queue_client.requests[0].waypoints[0]
    assert pose.header.frame_id == "world"
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (1.0, 2.0, 3.0)
    assert (pose.pose.orientation.z, pose.pose.orientation.w) == pytest.approx((sqrt(0.5), sqrt(0.5)))


def test_navigate_converts_ned_waypoint_batch_to_enu(monkeypatch) -> None:
    _patch_queue_spin(monkeypatch)
    node = FakeNode()
    navigate = NavigateSkill(node)
    assert navigate.call(((10.0, 20.0, -5.0, 0.0), (4.0, 2.0, 7.0, 450.0)), frame="NED") == 2
    first, second = node.queue_client.requests[0].waypoints
    assert (first.pose.position.x, first.pose.position.y, first.pose.position.z) == (20.0, 10.0, 5.0)
    assert (first.pose.orientation.z, first.pose.orientation.w) == pytest.approx((sqrt(0.5), sqrt(0.5)))
    assert (second.pose.position.x, second.pose.position.y, second.pose.position.z) == (2.0, 4.0, -7.0)
    assert (second.pose.orientation.z, second.pose.orientation.w) == pytest.approx((0.0, 1.0))


def test_navigate_clear_delegates_to_clear_service(monkeypatch) -> None:
    monkeypatch.setattr(
        "skills.primitives.clear_waypoints.rclpy.spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    navigate = NavigateSkill(FakeNode())
    assert navigate.clear() == 2


def test_navigate_reports_unavailable_queue_service() -> None:
    navigate = NavigateSkill(FakeNode(queue_available=False))
    with pytest.raises(SkillTimeoutError, match="unavailable"):
        navigate.call((1.0, 2.0, 3.0, 0.0))


@pytest.mark.parametrize(
    ("waypoints", "frame", "message"),
    [
        ((), "enu", "non-empty"),
        ((1.0, 2.0, 3.0), "enu", "one waypoint"),
        ((1.0, 2.0, 3.0, float("nan")), "enu", "finite"),
        ((1.0, 2.0, 3.0, 0.0), "ecef", "either 'enu' or 'ned'"),
    ],
)
def test_navigate_rejects_invalid_waypoint_input(waypoints, frame, message) -> None:
    with pytest.raises(ValueError, match=message):
        NavigateSkill(FakeNode()).call(waypoints, frame=frame)


def test_heading_normalization_and_coverage_output_compatibility() -> None:
    pose = pose_stamped_from_enu_waypoint((1.0, 2.0, 3.0, 450.0), frame_id="map")
    assert (pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w) == pytest.approx(
                enu_yaw_quaternion(90.0))

    from coverage_planner.ros_node import quaternion_from_compass_heading

    assert quaternion_from_compass_heading(0.0) == pytest.approx(enu_yaw_quaternion(90.0))
