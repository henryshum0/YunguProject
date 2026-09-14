"""Conversions between the detection core types and ROS messages.

The message layer is the interface a real detector has to meet, so it is kept
small and explicit: image-plane results are ``vision_msgs/Detection2DArray``,
world-frame results are ``vision_msgs/Detection3DArray``. A real RemDet node maps
onto the same messages directly — its per-detection ``(class_name, score, xyxy)``
becomes ``class_id``/``score`` plus the centre/size form used here.
"""

from __future__ import annotations

from builtin_interfaces.msg import Time
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from detection.camera import PixelBox
from detection.mock import MockDetection


def time_msg(stamp_sec: float) -> Time:
    """Convert seconds since the epoch into a ROS time message."""
    seconds = int(stamp_sec)
    nanoseconds = int(round((stamp_sec - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def time_seconds(stamp: Time) -> float:
    """Convert a ROS time message into seconds since the epoch."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pixel_box(bbox: BoundingBox2D) -> PixelBox:
    """Convert a ROS bounding box into the core's pixel box."""
    return PixelBox.from_center(
        bbox.center.position.x, bbox.center.position.y, bbox.size_x, bbox.size_y)


def detection_2d(detection: MockDetection, *, stamp: Time, frame_id: str) -> Detection2D:
    """Build one ``Detection2D`` from a mock detection."""
    message = Detection2D()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.id = detection.detection_id
    message.bbox.center.position.x = float(detection.box.center_x)
    message.bbox.center.position.y = float(detection.box.center_y)
    message.bbox.center.theta = 0.0
    message.bbox.size_x = float(detection.box.size_x)
    message.bbox.size_y = float(detection.box.size_y)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = detection.class_id
    hypothesis.hypothesis.score = float(detection.score)
    message.results = [hypothesis]
    return message


def detection_2d_array(
    detections, *, stamp_sec: float, frame_id: str,
) -> Detection2DArray:
    """Build the detector's per-frame output message."""
    stamp = time_msg(stamp_sec)
    message = Detection2DArray()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.detections = [
        detection_2d(detection, stamp=stamp, frame_id=frame_id) for detection in detections
    ]
    return message


def detection_3d(
    source: Detection2D,
    *,
    position: tuple[float, float, float],
    size: tuple[float, float],
    height: float,
    stamp: Time,
    frame_id: str,
) -> Detection3D:
    """Build a world-frame ``Detection3D`` from a localized image detection.

    ``id`` is carried over unchanged so a consumer can join the world position
    back onto the image-plane detection it came from.
    """
    message = Detection3D()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.id = source.id
    message.results = list(source.results)
    message.bbox.center.position.x = float(position[0])
    message.bbox.center.position.y = float(position[1])
    message.bbox.center.position.z = float(position[2])
    message.bbox.center.orientation.w = 1.0
    message.bbox.size.x = float(size[0])
    message.bbox.size.y = float(size[1])
    message.bbox.size.z = float(height)
    return message


def detection_3d_array(detections, *, stamp: Time, frame_id: str) -> Detection3DArray:
    """Wrap world-frame detections into their array message."""
    message = Detection3DArray()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.detections = list(detections)
    return message


def hypothesis_of(detection) -> tuple[str, float]:
    """Return the ``(class_id, score)`` of a detection's best hypothesis."""
    if not detection.results:
        return ("", 0.0)
    best = max(detection.results, key=lambda result: result.hypothesis.score)
    return (best.hypothesis.class_id, float(best.hypothesis.score))
