"""Detection skill: class selection, stream joining, and target confirmation."""

from __future__ import annotations

import pytest
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from skills import SkillExecutionError
from skills.skills.detect import Detection, DetectSkill, TargetAggregator, expand_classes
from skills.tests.config_data import TEST_CONFIG, TEST_CONFIG_WITH_DETECTION
from skills.tests.fakes import FakeNode


DETECTIONS = "/detection/detections"
DETECTIONS_WORLD = "/detection/detections_world"


# -- class selection --------------------------------------------------------
def test_group_names_expand_to_detector_classes() -> None:
    assert expand_classes(("person",)) == ("pedestrian", "people")
    assert expand_classes(("vehicle",)) == ("car", "van", "truck", "bus")
    assert expand_classes(("car",)) == ("car",)
    assert expand_classes("pedestrian") == ("pedestrian",)


def test_expansion_is_case_insensitive_and_deduplicated() -> None:
    assert expand_classes(("Person", "PEDESTRIAN", "people")) == ("pedestrian", "people")


def test_unknown_or_empty_classes_are_rejected() -> None:
    with pytest.raises(ValueError, match="unknown target class 'plane'"):
        expand_classes(("plane",))
    with pytest.raises(ValueError, match="at least one target class"):
        expand_classes(())
    with pytest.raises(ValueError, match="must not be empty"):
        expand_classes(("  ",))


# -- target confirmation ----------------------------------------------------
def detection(x: float, y: float, *, class_id="car", score=0.8, stamp=100.0,
              identifier="d") -> Detection:
    return Detection(detection_id=identifier, class_id=class_id, score=score,
                     bbox=(320.0, 240.0, 40.0, 20.0), stamp_sec=stamp,
                     world_position=(x, y, -1.65))


def test_a_target_is_confirmed_only_after_enough_consistent_hits() -> None:
    aggregator = TargetAggregator(min_hits=3, cluster_radius_m=6.0)
    assert aggregator.add(detection(10.0, 5.0)) is None
    assert aggregator.add(detection(10.4, 5.2)) is None
    assert aggregator.confirmed == ()
    confirmed = aggregator.add(detection(9.7, 4.8))
    assert confirmed is not None
    assert confirmed.hits == 3
    assert confirmed.position[0] == pytest.approx(10.03, abs=0.01)
    assert len(aggregator.confirmed) == 1


def test_scattered_detections_never_confirm_a_target() -> None:
    """This is what keeps injected false positives out of a search result."""
    aggregator = TargetAggregator(min_hits=3, cluster_radius_m=6.0)
    for index, (x, y) in enumerate(((0.0, 0.0), (40.0, 0.0), (0.0, 40.0), (-40.0, 20.0))):
        assert aggregator.add(detection(x, y, stamp=100.0 + index)) is None
    assert aggregator.confirmed == ()


def test_stale_clusters_expire_so_they_cannot_slowly_accumulate() -> None:
    aggregator = TargetAggregator(min_hits=3, cluster_radius_m=6.0, cluster_timeout_sec=5.0)
    aggregator.add(detection(10.0, 5.0, stamp=100.0))
    aggregator.add(detection(10.0, 5.0, stamp=101.0))
    # The next hit arrives long after the cluster went stale, so it starts over.
    assert aggregator.add(detection(10.0, 5.0, stamp=200.0)) is None
    assert aggregator.confirmed == ()


def test_low_scores_and_missing_world_positions_are_ignored() -> None:
    aggregator = TargetAggregator(min_hits=1, min_score=0.5)
    assert aggregator.add(detection(1.0, 1.0, score=0.2)) is None
    unplaced = Detection(detection_id="d", class_id="car", score=0.9,
                         bbox=(0.0, 0.0, 1.0, 1.0), stamp_sec=100.0, world_position=None)
    assert aggregator.add(unplaced) is None
    assert aggregator.confirmed == ()


def test_a_confirmed_target_reports_its_most_reported_class() -> None:
    """Detector class confusion must not change what the target is called."""
    aggregator = TargetAggregator(min_hits=3, cluster_radius_m=6.0)
    aggregator.add(detection(10.0, 5.0, class_id="car"))
    aggregator.add(detection(10.1, 5.1, class_id="van"))
    aggregator.add(detection(10.2, 5.0, class_id="car"))
    assert aggregator.confirmed[0].class_id == "car"


# -- the ROS-facing skill ---------------------------------------------------
def detection_2d_message(*entries, stamp_sec=100.0) -> Detection2DArray:
    message = Detection2DArray()
    message.header.stamp.sec = int(stamp_sec)
    message.header.frame_id = "front_camera_link"
    for identifier, class_id, score in entries:
        item = Detection2D()
        item.id = identifier
        item.bbox.center.position.x = 320.0
        item.bbox.center.position.y = 240.0
        item.bbox.size_x = 40.0
        item.bbox.size_y = 20.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = score
        item.results = [hypothesis]
        message.detections.append(item)
    return message


def detection_3d_message(*entries, stamp_sec=100.0) -> Detection3DArray:
    message = Detection3DArray()
    message.header.stamp.sec = int(stamp_sec)
    message.header.frame_id = "map"
    for identifier, position in entries:
        item = Detection3D()
        item.id = identifier
        item.bbox.center.position.x, item.bbox.center.position.y, item.bbox.center.position.z = position
        message.detections.append(item)
    return message


def started_skill(node: FakeNode, classes=("car",), **kwargs) -> DetectSkill:
    skill = DetectSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    skill.start(classes, **kwargs)
    return skill


def test_detect_skill_needs_a_detection_configuration() -> None:
    with pytest.raises(SkillExecutionError, match="no detection configuration"):
        DetectSkill(FakeNode(), config=TEST_CONFIG)


def test_the_skill_subscribes_to_both_detection_streams() -> None:
    node = FakeNode()
    skill = DetectSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    assert skill.name == "detect"
    assert set(node.subscriptions) == {DETECTIONS, DETECTIONS_WORLD}
    assert skill.detections_topic == DETECTIONS


def test_world_positions_are_joined_onto_their_image_detections() -> None:
    node = FakeNode()
    skill = started_skill(node)
    node.deliver(DETECTIONS, detection_2d_message(("car_01#1", "car", 0.8)))
    node.deliver(DETECTIONS_WORLD, detection_3d_message(("car_01#1", (11.9, 6.1, -1.65))))
    node.advance(0.3)

    collected = skill.collect()
    assert len(collected) == 1
    assert collected[0].detection_id == "car_01#1"
    assert collected[0].world_position == pytest.approx((11.9, 6.1, -1.65))
    assert collected[0].bbox == (320.0, 240.0, 40.0, 20.0)
    assert skill.frames_observed == 1
    assert skill.localizer_seen


def test_detections_are_reported_without_a_localizer() -> None:
    node = FakeNode()
    skill = started_skill(node)
    node.deliver(DETECTIONS, detection_2d_message(("car_01#1", "car", 0.8)))
    node.advance(0.3)

    collected = skill.collect()
    assert len(collected) == 1
    assert collected[0].world_position is None
    assert not skill.localizer_seen


def test_detections_wait_for_their_world_position_before_being_reported() -> None:
    node = FakeNode()
    skill = started_skill(node)
    node.deliver(DETECTIONS, detection_2d_message(("car_01#1", "car", 0.8)))
    assert skill.collect() == ()  # still inside the join grace period
    node.advance(0.3)
    assert len(skill.collect()) == 1


def test_other_classes_and_low_scores_are_filtered_out() -> None:
    node = FakeNode()
    skill = started_skill(node, ("person",), min_score=0.5)
    node.deliver(DETECTIONS, detection_2d_message(
        ("ped_01#1", "pedestrian", 0.8),
        ("ped_02#1", "pedestrian", 0.2),
        ("car_01#1", "car", 0.9),
    ))
    node.advance(0.3)
    collected = skill.collect()
    assert [item.detection_id for item in collected] == ["ped_01#1"]


def test_messages_before_start_and_after_stop_are_dropped() -> None:
    node = FakeNode()
    skill = DetectSkill(node, config=TEST_CONFIG_WITH_DETECTION)
    node.deliver(DETECTIONS, detection_2d_message(("car_01#0", "car", 0.8)))
    skill.start(("car",))
    node.deliver(DETECTIONS, detection_2d_message(("car_01#1", "car", 0.8)))
    skill.stop()
    node.deliver(DETECTIONS, detection_2d_message(("car_01#2", "car", 0.8)))
    node.advance(0.3)
    assert [item.detection_id for item in skill.collect()] == ["car_01#1"]


def test_start_clears_what_a_previous_run_left_behind() -> None:
    node = FakeNode()
    skill = started_skill(node)
    node.deliver(DETECTIONS, detection_2d_message(("car_01#1", "car", 0.8)))
    skill.start(("car",))
    node.advance(0.3)
    assert skill.collect() == ()
    assert skill.frames_observed == 0
