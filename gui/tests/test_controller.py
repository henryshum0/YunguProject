from __future__ import annotations

from pathlib import Path as FilePath

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from gui.controller import (
    ConnectionSettings,
    SkillController,
    format_path,
    format_progress,
    format_search_result,
)
from detection.truth import TruthMatch
from skills.detect import ConfirmedTarget
from skills.search import SearchProgress, SearchResult
from skills.config import CoveragePlannerSkillConfig, OffboardSkillConfig, SkillRuntimeConfig


SETTINGS = ConnectionSettings(
    config=SkillRuntimeConfig(
        offboard=OffboardSkillConfig(
            frame_id="map", queue_service="/queue", clear_service="/clear",
            takeoff_topic="/takeoff", land_topic="/land", queue_status_topic="/status"),
        coverage_planner=CoveragePlannerSkillConfig(
            frame_id="map", plan_service="/planner", planner_config_file=FilePath("/tmp/test-planner.json")),
    ),
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
    """The GUI uses the command-level entry point, not the blocking mission."""

    instances = []

    def __init__(self, node, **kwargs) -> None:
        super().__init__(node, **kwargs)
        self.missions = []
        self.__class__.instances.append(self)

    def plan_and_queue(self, corners, *, timeout_sec):
        return self.call(corners, timeout_sec=timeout_sec)

    def call(self, corners, *, classes=None, timeout_sec=None, mission_timeout_sec=None,
             stop_on_first_detection=None, stop_requested=None, on_progress=None,
             publish_result=False):
        if classes is None:   # plan_and_queue path
            return super().call(corners, publish_result=publish_result, timeout_sec=timeout_sec)
        self.missions.append({
            "corners": corners, "classes": classes, "timeout_sec": timeout_sec,
            "mission_timeout_sec": mission_timeout_sec,
            "stop_on_first_detection": stop_on_first_detection,
            "stop_requested": stop_requested, "on_progress": on_progress,
        })
        return MISSION_RESULT


TARGET = ConfirmedTarget(class_id="car", position=(11.9, 6.1, -1.65), hits=12,
                         best_score=0.83, first_seen_sec=10.0, last_seen_sec=25.0)
MISSION_RESULT = SearchResult(
    success=True, message="found 1 target(s) after 3/14 waypoint(s)", found=True,
    targets=(TARGET,), waypoints_total=14, waypoints_completed=3,
    termination_reason="found", path=Path(), detector_frames=180)


def test_controller_runs_the_mission_with_the_operator_settings(monkeypatch) -> None:
    import gui.controller as controller_module

    FakeSearchSkill.instances.clear()
    monkeypatch.setattr(controller_module, "SearchSkill", FakeSearchSkill)
    stop = object()
    progress = object()
    result = SkillController(FakeNode()).run_search_mission(
        ((0.0, 0.0),) * 4, classes=("vehicle",), settings=SETTINGS,
        mission_timeout_sec=900.0, stop_on_first_detection=True,
        stop_requested=stop, on_progress=progress)

    assert result is MISSION_RESULT
    mission = FakeSearchSkill.instances[0].missions[0]
    assert mission["classes"] == ("vehicle",)
    assert mission["mission_timeout_sec"] == 900.0
    assert mission["stop_on_first_detection"] is True
    assert mission["timeout_sec"] == SETTINGS.timeout_sec
    # The mission must stay steerable from the GUI while it runs.
    assert mission["stop_requested"] is stop
    assert mission["on_progress"] is progress


def test_format_search_result_reports_the_mission_conclusion() -> None:
    rendered = format_search_result(MISSION_RESULT)
    assert MISSION_RESULT.message in rendered
    assert "termination_reason : found" in rendered
    assert "route progress     : 3/14 (21%)" in rendered
    assert "car" in rendered and "11.90" in rendered
    assert "hits=12" in rendered


def test_format_search_result_shows_the_truth_next_to_the_estimate() -> None:
    """The reported position is an estimate; in simulation the truth is printed with it."""
    match = TruthMatch(target_id="car_03", class_id="car",
                       position=(52.0, -2.0, -1.65), error_m=2.43)
    rendered = format_search_result(MISSION_RESULT, (match,))
    assert "estimate (   11.90,     6.10" in rendered
    assert "truth    (   52.00,    -2.00" in rendered
    assert "car_03 [car]" in rendered
    assert "error 2.43 m" in rendered


def test_format_search_result_flags_an_estimate_with_no_target_nearby() -> None:
    rendered = format_search_result(MISSION_RESULT, (None,))
    assert "no target nearby (possible false positive)" in rendered


def test_format_search_result_omits_truth_when_it_is_unavailable() -> None:
    rendered = format_search_result(MISSION_RESULT)
    assert "estimate" in rendered
    assert "truth" not in rendered


def test_format_search_result_says_so_when_nothing_was_found() -> None:
    empty = SearchResult(
        success=True, message="covered 14/14 waypoint(s), no car/van/truck/bus found",
        found=False, targets=(), waypoints_total=14, waypoints_completed=14,
        termination_reason="covered", path=Path(), detector_frames=2618)
    assert "targets: none confirmed" in format_search_result(empty)


def test_format_progress_summarizes_a_mission_in_flight() -> None:
    running = format_progress(SearchProgress(
        waypoints_total=14, waypoints_remaining=11, targets=(TARGET,),
        detector_frames=180, elapsed_sec=42.0))
    assert "3/14 waypoint(s)" in running
    assert "180 detector frame(s)" in running
    assert "car at (11.9, 6.1)" in running

    waiting = format_progress(SearchProgress(
        waypoints_total=14, waypoints_remaining=None, targets=(),
        detector_frames=0, elapsed_sec=1.0))
    assert "queue not reported yet" in waiting
    assert "found: none yet" in waiting


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
    assert FakeNavigateSkill.instances[0].kwargs == {"config": SETTINGS.config}
    assert controller.clear(SETTINGS) == 3
    assert isinstance(controller.plan_search(((0.0, 0.0),) * 4, settings=SETTINGS), Path)
    assert isinstance(controller.search_and_queue(((0.0, 0.0),) * 4, settings=SETTINGS), Path)


def test_flight_publishers_can_be_created_before_a_long_action() -> None:
    node = FakeNode()
    controller = SkillController(node)
    controller.prepare_flight_commands(SETTINGS)
    assert set(node.publishers) == {"/takeoff", "/land"}

    # Publishing later must reuse them rather than adding entities to a node the
    # worker thread is already spinning.
    controller.land(SETTINGS)
    assert len(node.publishers) == 2
    assert node.publishers["/land"].messages[0].data is True


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
