from __future__ import annotations

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from gui.controller import ConnectionSettings, SkillController, format_path


SETTINGS = ConnectionSettings(
    frame_id="map",
    planner_service="/planner",
    queue_service="/queue",
    clear_service="/clear",
    takeoff_topic="/takeoff",
    land_topic="/land",
    timeout_sec=4.0,
)


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.publishers = {}

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


class FakeNavigateSkill:
    instances = []

    def __init__(self, node, **kwargs) -> None:
        self.kwargs = kwargs
        self.calls = []
        self.__class__.instances.append(self)

    def call(self, waypoints, *, frame, timeout_sec):
        self.calls.append((waypoints, frame, timeout_sec))
        return len(waypoints)

    def clear(self, *, timeout_sec):
        self.calls.append(("clear", timeout_sec))
        return 3


class FakePlanSearchPrimitive:
    def __init__(self, node, **kwargs) -> None:
        self.kwargs = kwargs

    def call(self, corners, *, publish_result=False, timeout_sec):
        path = Path()
        path.header.frame_id = "map"
        path.poses.append(PoseStamped())
        return path


class FakeSearchSkill(FakePlanSearchPrimitive):
    pass


def test_controller_uses_settings_for_skills_and_flight_topics(monkeypatch) -> None:
    import gui.controller as controller_module

    FakeNavigateSkill.instances.clear()
    monkeypatch.setattr(controller_module, "NavigateSkill", FakeNavigateSkill)
    monkeypatch.setattr(controller_module, "PlanSearchPrimitive", FakePlanSearchPrimitive)
    monkeypatch.setattr(controller_module, "SearchSkill", FakeSearchSkill)
    node = FakeNode()
    controller = SkillController(node)

    controller.takeoff(SETTINGS)
    controller.land(SETTINGS)
    assert node.publishers["/takeoff"].messages[0].data is True
    assert node.publishers["/land"].messages[0].data is True

    assert controller.navigate(((1.0, 2.0, 3.0, 0.0),), frame="enu", settings=SETTINGS) == 1
    assert FakeNavigateSkill.instances[0].kwargs == {
        "frame_id": "map", "queue_service": "/queue", "clear_service": "/clear"}
    assert controller.clear(SETTINGS) == 3
    assert isinstance(controller.plan_search(((0.0, 0.0),) * 4, settings=SETTINGS), Path)
    assert isinstance(controller.search_and_queue(((0.0, 0.0),) * 4, settings=SETTINGS), Path)


def test_controller_propagates_skill_errors(monkeypatch) -> None:
    import gui.controller as controller_module

    class FailingNavigate(FakeNavigateSkill):
        def call(self, waypoints, *, frame, timeout_sec):
            raise RuntimeError("queue unavailable")

    monkeypatch.setattr(controller_module, "NavigateSkill", FailingNavigate)
    with pytest.raises(RuntimeError, match="unavailable"):
        SkillController(FakeNode()).navigate(((1.0, 2.0, 3.0, 0.0),), frame="enu", settings=SETTINGS)


def test_format_path_reports_enu_pose_coordinates() -> None:
    path = Path()
    path.header.frame_id = "map"
    pose = PoseStamped()
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = 3.0
    pose.pose.orientation.w = 1.0
    path.poses.append(pose)
    assert format_path(path) == "1 waypoint(s), frame=map\n001: x=1.00, y=2.00, z=3.00, yaw=0.0 deg"
