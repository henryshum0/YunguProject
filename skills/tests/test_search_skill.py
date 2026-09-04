from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from skills import SearchSkill


class FakeNode:
    pass


class FakePlanSearchPrimitive:
    instances = []

    def __init__(self, node, *, frame_id, service_name) -> None:
        self.frame_id = frame_id
        self.service_name = service_name
        self.requests = []
        self.publish_results = []
        self.timeouts = []
        self.path = Path()
        self.path.header.frame_id = frame_id
        self.path.poses = [PoseStamped(), PoseStamped()]
        self.__class__.instances.append(self)

    def call(self, request, *, publish_result, timeout_sec):
        self.requests.append(request)
        self.publish_results.append(publish_result)
        self.timeouts.append(timeout_sec)
        return self.path


class FakeNavigateSkill:
    instances = []

    def __init__(self, node, *, frame_id, queue_service, clear_service) -> None:
        self.frame_id = frame_id
        self.queue_service = queue_service
        self.clear_service = clear_service
        self.poses = []
        self.timeouts = []
        self.__class__.instances.append(self)

    def call_poses(self, poses, *, timeout_sec):
        self.poses.append(tuple(poses))
        self.timeouts.append(timeout_sec)
        return len(poses)


def test_search_skill_plans_then_queues_path_through_navigate(monkeypatch) -> None:
    FakePlanSearchPrimitive.instances.clear()
    FakeNavigateSkill.instances.clear()
    monkeypatch.setattr("skills.search.PlanSearchPrimitive", FakePlanSearchPrimitive)
    monkeypatch.setattr("skills.search.NavigateSkill", FakeNavigateSkill)

    skill = SearchSkill(
        FakeNode(), frame_id="world", service_name="/planner", queue_service="/goals")
    corners = ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0))
    result = skill.call(corners, timeout_sec=3.0)

    planner = FakePlanSearchPrimitive.instances[0]
    navigate = FakeNavigateSkill.instances[0]
    assert skill.name == "search"
    assert skill.service_name == "/planner"
    assert skill.queue_service == "/goals"
    assert result is planner.path
    assert planner.requests == [corners]
    assert planner.publish_results == [True]
    assert planner.timeouts == [3.0]
    assert navigate.poses == [tuple(result.poses)]
    assert navigate.timeouts == [3.0]
