"""ROS node that draws detection boxes onto the camera stream.

A debug view, and nothing depends on it — but it is the fastest way to see
whether the detection loop is actually working, and it reads the detector's
public topic, so it shows a real detector's output just as well as the mock's.

It runs as its own node on purpose. Decoding a 640x480 stream costs a full-frame
copy per published overlay, and inside the detector that competes with the
detector's own timing; in a separate process it cannot.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from detection.config import DetectionConfig, DetectionConfigError
from detection.overlay import ImageOverlayError, OverlayBox, decode, draw, encode
from detection.ros.conversions import hypothesis_of


class DetectionOverlayNode(Node):
    """Republish the camera stream with the latest detection boxes drawn on it."""

    def __init__(self) -> None:
        super().__init__("detection_overlay")
        self.declare_parameter("detection_config", "")
        config_path = str(self.get_parameter("detection_config").value).strip()
        if not config_path:
            raise DetectionConfigError("parameter 'detection_config' is required")
        self._config = DetectionConfig.load(Path(config_path))

        topics = self._config.topics
        self._boxes: tuple[OverlayBox, ...] = ()
        self._last_published_sec = 0.0
        self._publisher = self.create_publisher(
            Image, topics.image_overlay, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, topics.detections, self._on_detections,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Image, topics.image_in, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"detection_overlay: '{topics.image_in}' + '{topics.detections}' -> "
            f"'{topics.image_overlay}' at up to {self._config.overlay.max_rate_hz:.1f} Hz")

    def _on_detections(self, message: Detection2DArray) -> None:
        boxes = []
        for detection in message.detections:
            class_id, score = hypothesis_of(detection)
            half_x = detection.bbox.size_x / 2.0
            half_y = detection.bbox.size_y / 2.0
            centre = detection.bbox.center.position
            boxes.append(OverlayBox(
                min_x=centre.x - half_x, min_y=centre.y - half_y,
                max_x=centre.x + half_x, max_y=centre.y + half_y,
                class_id=class_id, score=score))
        self._boxes = tuple(boxes)

    def _on_image(self, message: Image) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_published_sec < 1.0 / self._config.overlay.max_rate_hz:
            return
        self._last_published_sec = now
        try:
            image = decode(message.width, message.height, message.step,
                           message.encoding, message.data)
        except ImageOverlayError as error:
            self.get_logger().warn(f"overlay skipped: {error}", throttle_duration_sec=10.0)
            return
        draw(image, self._boxes)
        overlay = Image()
        overlay.header = message.header
        overlay.height = message.height
        overlay.width = message.width
        overlay.encoding = message.encoding
        overlay.is_bigendian = message.is_bigendian
        overlay.step = message.width * 3
        overlay.data = encode(image, message.encoding)
        self._publisher.publish(overlay)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        node = DetectionOverlayNode()
    except DetectionConfigError as error:
        print(f"detection_overlay: {error}")
        rclpy.shutdown()
        raise SystemExit(1) from error
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
