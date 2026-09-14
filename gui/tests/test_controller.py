from __future__ import annotations

from pathlib import Path as FilePath

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from gui.skill_interfaces.controller import ConnectionSettings, SkillController, format_path
from skills.config import CoveragePlannerSkillConfig, OffboardSkillConfig, SkillRuntimeConfig


SETTINGS = ConnectionSettings(
    config=SkillRuntimeConfig(
        offboard=OffboardSkillConfig(
            frame_id="map", queue_service="/queue", clear_service="/clear",
            takeoff_service="/takeoff", land_service="/land", queue_status_topic="/status"),
        coverage_planner=CoveragePlannerSkillConfig(
            frame_id="map", plan_service="/planner", planner_config_file=FilePath("/tmp/test-planner.json")),
    ),
    timeout_sec=4.0,
)


class FakeNode:
    def create_publisher(self, *_args, **_kwargs):
        raise AssertionError("flight commands must use services, not publishers")


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


class FakeFlightPrimitive:
    instances = []

    def __init__(self, node, **kwargs) -> None:
        self.kwargs = kwargs
        self.calls = []
        self.__class__.instances.append(self)

    def call(self, *, timeout_sec):
        self.calls.append(timeout_sec)
        return "accepted"


def test_controller_uses_settings_for_skills_and_flight_services(monkeypatch) -> None:
    import gui.skill_interfaces.controller as controller_module

    FakeNavigateSkill.instances.clear()
    monkeypatch.setattr(controller_module, "NavigateSkill", FakeNavigateSkill)
    monkeypatch.setattr(controller_module, "PlanSearchPrimitive", FakePlanSearchPrimitive)
    monkeypatch.setattr(controller_module, "SearchSkill", FakeSearchSkill)
    monkeypatch.setattr(controller_module, "TakeoffPrimitive", FakeFlightPrimitive)
    monkeypatch.setattr(controller_module, "LandPrimitive", FakeFlightPrimitive)
    node = FakeNode()
    controller = SkillController(node)

    FakeFlightPrimitive.instances.clear()
    assert controller.takeoff(SETTINGS) == "accepted"
    assert controller.land(SETTINGS) == "accepted"
    assert [primitive.kwargs for primitive in FakeFlightPrimitive.instances] == [
        {"config": SETTINGS.config}, {"config": SETTINGS.config}]
    assert [primitive.calls for primitive in FakeFlightPrimitive.instances] == [[4.0], [4.0]]

    assert controller.navigate(((1.0, 2.0, 3.0, 0.0),), frame="enu", settings=SETTINGS) == 1
    assert FakeNavigateSkill.instances[0].kwargs == {"config": SETTINGS.config}
    assert controller.clear(SETTINGS) == 3
    assert isinstance(controller.plan_search(((0.0, 0.0),) * 4, settings=SETTINGS), Path)
    assert isinstance(controller.search_and_queue(((0.0, 0.0),) * 4, settings=SETTINGS), Path)


def test_controller_propagates_skill_errors(monkeypatch) -> None:
    import gui.skill_interfaces.controller as controller_module

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
