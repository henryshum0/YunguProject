"""The ROS message layer a real detector has to match."""

from __future__ import annotations

import pytest
from vision_msgs.msg import Detection2DArray, Detection3DArray

from detection.camera import PixelBox
from detection.mock import MockDetection
from detection.ros.conversions import (
    detection_2d_array,
    detection_3d,
    detection_3d_array,
    hypothesis_of,
    pixel_box,
    time_msg,
    time_seconds,
)


def mock_detection(detection_id: str = "car_01#000001") -> MockDetection:
    return MockDetection(
        detection_id=detection_id, class_id="car", score=0.84,
        box=PixelBox.from_center(320.0, 240.0, 40.0, 20.0),
        target_id="car_01", range_m=18.0)


def test_time_round_trip_keeps_sub_millisecond_precision() -> None:
    for seconds in (0.0, 1.5, 1789112372.123456):
        assert time_seconds(time_msg(seconds)) == pytest.approx(seconds, abs=1e-6)
    # Rounding up to a full second must carry into the seconds field.
    assert time_msg(1.9999999999).nanosec < 1_000_000_000


def test_a_mock_detection_becomes_a_detection_2d_array() -> None:
    message = detection_2d_array(
        (mock_detection(),), stamp_sec=1000.25, frame_id="front_camera_link")
    assert isinstance(message, Detection2DArray)
    assert message.header.frame_id == "front_camera_link"
    assert time_seconds(message.header.stamp) == pytest.approx(1000.25)

    detection = message.detections[0]
    assert detection.id == "car_01#000001"
    assert detection.bbox.center.position.x == 320.0
    assert detection.bbox.size_x == 40.0
    assert hypothesis_of(detection) == ("car", pytest.approx(0.84))
    assert pixel_box(detection.bbox) == PixelBox.from_center(320.0, 240.0, 40.0, 20.0)


def test_a_world_detection_keeps_the_identity_and_hypothesis_of_its_source() -> None:
    source = detection_2d_array(
        (mock_detection(),), stamp_sec=5.0, frame_id="front_camera_link").detections[0]
    placed = detection_3d(source, position=(11.9, 6.1, -1.65), size=(2.0, 2.0), height=0.0,
                          stamp=time_msg(5.0), frame_id="map")
    assert placed.id == source.id           # the join key back to the image box
    assert hypothesis_of(placed) == ("car", pytest.approx(0.84))
    assert placed.bbox.center.position.x == pytest.approx(11.9)
    assert placed.bbox.center.orientation.w == 1.0
    assert placed.bbox.size.z == 0.0        # height is not observable from one box

    array = detection_3d_array((placed,), stamp=time_msg(5.0), frame_id="map")
    assert isinstance(array, Detection3DArray)
    assert array.header.frame_id == "map"
    assert len(array.detections) == 1


def test_a_detection_without_hypotheses_reports_nothing() -> None:
    empty = detection_2d_array((), stamp_sec=0.0, frame_id="camera")
    assert empty.detections == []
    source = detection_2d_array((mock_detection(),), stamp_sec=0.0, frame_id="c").detections[0]
    source.results = []
    assert hypothesis_of(source) == ("", 0.0)
