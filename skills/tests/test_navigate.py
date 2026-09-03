from __future__ import annotations

from math import sqrt

import pytest

from skills import NavigateSkill
from skills.frames import enu_yaw_quaternion, pose_stamped_from_enu_waypoint


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.publisher = FakePublisher()
        self.publisher_topic = None

    def create_publisher(self, message_type, topic, qos):
        self.publisher_topic = topic
        return self.publisher


def test_navigate_converts_single_enu_waypoint_and_publishes() -> None:
    node = FakeNode()
    navigate = NavigateSkill(node, frame_id="world")
    assert navigate.name == "navigate"
    assert navigate.call((1.0, 2.0, 3.0, 90.0)) == 1
    assert node.publisher_topic == "/waypoint_buffer"
    pose = node.publisher.messages[0]
    assert pose.header.frame_id == "world"
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (1.0, 2.0, 3.0)
    assert (pose.pose.orientation.z, pose.pose.orientation.w) == pytest.approx((sqrt(0.5), sqrt(0.5)))


def test_navigate_converts_ned_waypoint_batch_to_enu() -> None:
    node = FakeNode()
    navigate = NavigateSkill(node)
    assert navigate.call(((10.0, 20.0, -5.0, 0.0), (4.0, 2.0, 7.0, 450.0)), frame="NED") == 2
    first, second = node.publisher.messages
    assert (first.pose.position.x, first.pose.position.y, first.pose.position.z) == (20.0, 10.0, 5.0)
    assert (first.pose.orientation.z, first.pose.orientation.w) == pytest.approx((sqrt(0.5), sqrt(0.5)))
    assert (second.pose.position.x, second.pose.position.y, second.pose.position.z) == (2.0, 4.0, -7.0)
    assert (second.pose.orientation.z, second.pose.orientation.w) == pytest.approx((0.0, 1.0))


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
