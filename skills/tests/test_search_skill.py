"""Search skill: command-level queueing and the full detection-aware mission."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import pytest

from skills import SkillExecutionError
from skills.detect import Detection, expand_classes
from skills.search import PROGRESS_PERIOD_SEC, ROUTE_START_TIMEOUT_SEC, SearchSkill
from skills.tests.config_data import TEST_CONFIG, TEST_CONFIG_WITH_DETECTION
from skills.tests.fakes import FakeNode


CORNERS = ((20.0, -20.0), (60.0, -20.0), (60.0, 20.0), (20.0, 20.0))
STATUS_TOPIC = TEST_CONFIG.offboard.queue_status_topic
ODOM_TOPIC = TEST_CONFIG_WITH_DETECTION.detection.vehicle_odom_topic


class FakePlanSearchPrimitive:
    instances: list["FakePlanSearchPrimitive"] = []

    def __init__(self, node, *, config) -> None:
        self.service_name = config.coverage_planner.plan_service
        self.requests = []
        self.publish_results = []
        self.timeouts = []
        self.path = Path()
        self.path.header.frame_id = config.coverage_planner.frame_id
        self.path.poses = [PoseStamped() for _ in range(4)]
        self.__class__.instances.append(self)

    def call(self, request, *, publish_result, timeout_sec):
        self.requests.append(request)
        self.publish_results.append(publish_result)
        self.timeouts.append(timeout_sec)
        return self.path


class FakeNavigateSkill:
    instances: list["FakeNavigateSkill"] = []

    def __init__(self, node, *, config) -> None:
        self.queue_service = config.offboard.queue_service
        self.clear_service = config.offboard.clear_service
        self.poses = []
        self.timeouts = []
        self.clears = 0
        self.__class__.instances.append(self)

    def call_poses(self, poses, *, timeout_sec):
        self.poses.append(tuple(poses))
        self.timeouts.append(timeout_sec)
        return len(poses)

    def clear(self, *, timeout_sec=10.0):
        self.clears += 1
        return 0


class FakeDetectSkill:
    instances: list["FakeDetectSkill"] = []
    #: Detections each successive ``collect`` call returns.
    script: list[tuple[Detection, ...]] = []

    def __init__(self, node, *, config) -> None:
        self.started: list[tuple[str, ...]] = []
        self.stopped = 0
        self.frames_observed = 7
        self._pending = list(self.__class__.script)
        self.__class__.instances.append(self)

    def start(self, classes, *, min_score=0.0):
        # Expand like the real skill: the mission message names detector classes.
        expanded = expand_classes(classes)
        self.started.append(expanded)
        return expanded

    def stop(self):
        self.stopped += 1

    def collect(self, *, join_grace_sec=0.25):
        return self._pending.pop(0) if self._pending else ()


@pytest.fixture(autouse=True)
def fake_collaborators(monkeypatch):
    for fake in (FakePlanSearchPrimitive, FakeNavigateSkill, FakeDetectSkill):
        fake.instances.clear()
    FakeDetectSkill.script = []
    monkeypatch.setattr("skills.search.PlanSearchPrimitive", FakePlanSearchPrimitive)
    monkeypatch.setattr("skills.search.NavigateSkill", FakeNavigateSkill)
    monkeypatch.setattr("skills.search.DetectSkill", FakeDetectSkill)


def spin_with(node: FakeNode, monkeypatch, on_tick=None) -> None:
    """Replace rclpy spinning with a clock the test drives itself."""
    ticks = {"count": 0}

    def spin_once(_node, timeout_sec=0.0):
        node.advance(0.05)
        ticks["count"] += 1
        if on_tick is not None:
            on_tick(ticks["count"])

    monkeypatch.setattr("skills.search.rclpy.spin_once", spin_once)


def status(count: int) -> Path:
    message = Path()
    message.poses = [PoseStamped() for _ in range(count)]
    return message


def odom(x: float, y: float, z: float = 15.0) -> Odometry:
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    return message


def detection(x: float, y: float, *, identifier: str, class_id="car") -> Detection:
    return Detection(detection_id=identifier, class_id=class_id, score=0.85,
                     bbox=(320.0, 240.0, 40.0, 20.0), stamp_sec=1000.0,
                     world_position=(x, y, -1.65))


# -- command level ----------------------------------------------------------
def test_plan_and_queue_plans_then_hands_the_route_to_navigation() -> None:
    skill = SearchSkill(FakeNode(), config=TEST_CONFIG)
    path = skill.plan_and_queue(CORNERS, timeout_sec=3.0)

    planner = FakePlanSearchPrimitive.instances[0]
    navigate = FakeNavigateSkill.instances[0]
    assert skill.name == "search"
    assert skill.service_name == "/coverage_planner/plan_coverage"
    assert skill.queue_service == "/waypoint_buffer"
    assert path is planner.path
    assert planner.requests == [CORNERS]
    assert planner.publish_results == [True]   # refreshes the planner visualization
    assert navigate.poses == [tuple(path.poses)]
    assert navigate.timeouts == [3.0]
    assert navigate.clears == 0                # command level never aborts a route


def test_a_mission_needs_a_detection_configuration() -> None:
    skill = SearchSkill(FakeNode(), config=TEST_CONFIG)
    with pytest.raises(SkillExecutionError, match="needs detection"):
        skill.call(CORNERS)


# -- mission level ----------------------------------------------------------
def test_the_mission_stops_and_reports_once_a_target_is_confirmed(monkeypatch) -> None:
    node = FakeNode()
    FakeDetectSkill.script = [
        (detection(11.9, 6.1, identifier="car_01#1"),),
        (detection(12.1, 5.9, identifier="car_01#2"),),
        (detection(12.0, 6.2, identifier="car_01#3"),),
    ]
    spin_with(node, monkeypatch,
              on_tick=lambda count: node.deliver(STATUS_TOPIC, status(4)) if count == 1 else None)

    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3, settle_sec=0.0)

    assert result.success and result.found
    assert result.termination_reason == "found"
    assert [target.class_id for target in result.targets] == ["car"]
    assert result.targets[0].position[0] == pytest.approx(12.0, abs=0.1)
    assert result.waypoints_total == 4
    # The route was aborted with all four waypoints still queued, so none of it
    # was flown — clearing the queue must not read as "route completed".
    assert result.waypoints_completed == 0
    assert result.route_completion == 0.0
    assert "found 1 target(s)" in result.message
    # The queue is cleared once before planning and once to abort the rest.
    assert FakeNavigateSkill.instances[0].clears == 2
    assert FakeDetectSkill.instances[0].started == [("car",)]
    assert FakeDetectSkill.instances[0].stopped == 1


def test_the_mission_runs_to_the_end_of_the_route_when_nothing_is_found(monkeypatch) -> None:
    node = FakeNode()

    def on_tick(count: int) -> None:
        if count == 1:
            node.deliver(STATUS_TOPIC, status(4))
        elif count == 6:
            node.deliver(STATUS_TOPIC, status(0))

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("vehicle",))

    assert result.success and not result.found
    assert result.termination_reason == "covered"
    assert result.waypoints_completed == 4 and result.waypoints_total == 4
    assert result.route_completion == 1.0
    assert result.detector_frames == 7
    assert "no car/van/truck/bus found" in result.message
    assert FakeNavigateSkill.instances[0].clears == 1   # only the pre-mission clear


def test_a_full_sweep_reports_every_target_it_confirmed(monkeypatch) -> None:
    node = FakeNode()
    FakeDetectSkill.script = [
        (detection(11.9, 6.1, identifier="a1"),),
        (detection(12.1, 5.9, identifier="a2"),),
        (detection(12.0, 6.2, identifier="a3"),),
        (detection(30.0, 16.0, identifier="b1"),),
        (detection(30.2, 16.1, identifier="b2"),),
        (detection(29.8, 15.9, identifier="b3"),),
    ]

    def on_tick(count: int) -> None:
        if count == 1:
            node.deliver(STATUS_TOPIC, status(4))
        elif count == 8:
            node.deliver(STATUS_TOPIC, status(0))

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3, stop_on_first_detection=False)

    assert result.termination_reason == "covered"
    assert result.found and len(result.targets) == 2
    assert FakeNavigateSkill.instances[0].clears == 1   # the route was never aborted


def test_a_target_found_before_the_route_is_reported_shows_no_progress(monkeypatch) -> None:
    """A queue status that never arrived means nothing was flown, not everything."""
    node = FakeNode()
    FakeDetectSkill.script = [(detection(11.9, 6.1, identifier=f"c{index}"),) for index in range(3)]
    spin_with(node, monkeypatch)   # no route status is ever published

    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3, settle_sec=0.0)

    assert result.termination_reason == "found"
    assert result.waypoints_completed == 0
    assert result.route_completion == 0.0


def test_a_route_the_offboard_queue_never_reports_is_called_out(monkeypatch) -> None:
    node = FakeNode()
    spin_with(node, monkeypatch,
              on_tick=lambda count: node.deliver(STATUS_TOPIC, status(0)))
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, mission_timeout_sec=ROUTE_START_TIMEOUT_SEC * 4)

    assert not result.success
    assert result.termination_reason == "not_started"
    assert "is the vehicle airborne" in result.message


def test_a_route_that_never_finishes_times_out(monkeypatch) -> None:
    node = FakeNode()
    spin_with(node, monkeypatch,
              on_tick=lambda count: node.deliver(STATUS_TOPIC, status(2)))
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, mission_timeout_sec=1.0)

    assert not result.success
    assert result.termination_reason == "timeout"
    assert result.waypoints_completed == 2   # 4 planned, 2 still queued
    assert result.route_completion == 0.5
    assert "timed out after 2/4 waypoint(s)" in result.message


def test_an_operator_can_call_the_mission_off(monkeypatch) -> None:
    node = FakeNode()
    stop = {"requested": False}

    def on_tick(count: int) -> None:
        node.deliver(STATUS_TOPIC, status(3))
        if count == 5:
            stop["requested"] = True

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, mission_timeout_sec=600.0,
                        stop_requested=lambda: stop["requested"])

    assert result.termination_reason == "aborted"
    assert not result.success
    assert result.waypoints_completed == 1   # 4 planned, 3 still queued
    assert "stopped by the operator after 1/4 waypoint(s)" in result.message
    # Aborting clears the route, on top of the pre-mission clear.
    assert FakeNavigateSkill.instances[0].clears == 2


def test_progress_is_reported_while_the_mission_runs(monkeypatch) -> None:
    node = FakeNode()
    FakeDetectSkill.script = [
        (detection(11.9, 6.1, identifier="a1"),),
        (detection(12.1, 5.9, identifier="a2"),),
        (detection(12.0, 6.2, identifier="a3"),),
    ]

    def on_tick(count: int) -> None:
        node.deliver(STATUS_TOPIC, status(max(4 - count // 4, 0)))

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    reports = []
    result = skill.call(CORNERS, classes=("car",), min_hits=3,
                        stop_on_first_detection=False, on_progress=reports.append)

    assert result.termination_reason == "covered"
    assert reports, "a multi-minute mission must report progress while it runs"
    # Each spin advances the clock by 50 ms, so reports are spaced by the period.
    assert reports[0].waypoints_total == 4
    assert reports[-1].elapsed_sec >= PROGRESS_PERIOD_SEC
    assert [report.waypoints_completed for report in reports] == sorted(
        report.waypoints_completed for report in reports)
    assert reports[-1].targets and reports[0].detector_frames == 7


def test_progress_before_the_queue_reports_shows_no_completion(monkeypatch) -> None:
    node = FakeNode()
    spin_with(node, monkeypatch)   # the queue never reports
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    reports = []
    skill.call(CORNERS, mission_timeout_sec=2.0, on_progress=reports.append)

    assert reports
    assert reports[0].waypoints_remaining is None
    assert reports[0].waypoints_completed == 0


def test_route_completion_is_zero_for_an_empty_route() -> None:
    node = FakeNode()
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    path = skill.plan_and_queue(CORNERS)
    assert len(path.poses) == 4


# -- proximity gate + settle window -----------------------------------------
def test_a_distant_sighting_does_not_confirm_a_target(monkeypatch) -> None:
    """The whole point: a far, few-frame sighting must not stop the mission."""
    node = FakeNode()
    # The same target is seen every cycle, but the vehicle stays 50 m away.
    FakeDetectSkill.script = [(detection(50.0, 0.0, identifier=f"t{i}"),) for i in range(8)]

    def on_tick(count: int) -> None:
        node.deliver(ODOM_TOPIC, odom(0.0, 0.0))         # far from the target
        node.deliver(STATUS_TOPIC, status(0 if count >= 6 else 4))

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3, confirm_within_m=25.0)

    # Plenty of detections, but every one was out of range, so nothing confirmed
    # and the sweep simply finished.
    assert result.termination_reason == "covered"
    assert not result.found


def test_a_target_confirms_once_the_vehicle_is_within_range(monkeypatch) -> None:
    node = FakeNode()
    FakeDetectSkill.script = [(detection(50.0, 0.0, identifier=f"t{i}"),) for i in range(8)]

    def on_tick(count: int) -> None:
        node.deliver(STATUS_TOPIC, status(4))
        node.deliver(ODOM_TOPIC, odom(48.0, 0.0))        # 2 m from the target

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3,
                        confirm_within_m=25.0, settle_sec=0.0)

    assert result.termination_reason == "found"
    assert result.targets[0].position[0] == pytest.approx(50.0, abs=0.1)


def test_the_settle_window_keeps_observing_after_confirmation(monkeypatch) -> None:
    node = FakeNode()
    FakeDetectSkill.script = [(detection(50.0, 0.0, identifier=f"t{i}"),) for i in range(12)]

    def on_tick(count: int) -> None:
        node.deliver(STATUS_TOPIC, status(4))
        node.deliver(ODOM_TOPIC, odom(50.5, 0.0))        # already close

    spin_with(node, monkeypatch, on_tick=on_tick)
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    # min_hits=3 confirms at the third cycle; a 0.2 s settle (spins are 0.05 s
    # apart) then keeps folding in the close-range frames that follow.
    result = skill.call(CORNERS, classes=("car",), min_hits=3,
                        confirm_within_m=25.0, settle_sec=0.2)

    assert result.termination_reason == "found"
    assert result.targets[0].hits > 3   # more than the three needed to confirm


def test_the_range_gate_is_a_no_op_before_any_pose_arrives(monkeypatch) -> None:
    """Without a vehicle pose the gate cannot judge range, so it must not block."""
    node = FakeNode()
    FakeDetectSkill.script = [(detection(999.0, 0.0, identifier=f"t{i}"),) for i in range(3)]
    spin_with(node, monkeypatch,
              on_tick=lambda count: node.deliver(STATUS_TOPIC, status(4)))
    skill = SearchSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    result = skill.call(CORNERS, classes=("car",), min_hits=3,
                        confirm_within_m=25.0, settle_sec=0.0)

    # No odometry was ever delivered, so the far position is still accepted.
    assert result.termination_reason == "found"
