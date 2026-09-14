"""ROS node that gives image-plane detections a place on the map.

A detector reports pixels; a search skill needs metres. This node closes that
gap and is the one piece of the detection path that does *not* change when the
mock detector is replaced by the real one: it consumes the same
``vision_msgs/Detection2DArray`` either way.

Each box is back-projected through its ground-contact pixel onto the ground
plane, using the vehicle pose at the frame's capture time — looked up from a
buffer, because a detection always arrives after the moment it describes.
``Detection3D.id`` is carried over from the source detection so a consumer can
join the world position back onto the image-plane box it came from.
"""

from __future__ import annotations

from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray, Detection3DArray

from detection.config import DetectionConfig, DetectionConfigError
from detection.localization import PoseBuffer, TimedPose, ground_footprint, ground_position
from detection.ros.conversions import (
    detection_3d,
    detection_3d_array,
    hypothesis_of,
    pixel_box,
    time_seconds,
)


#: Seconds a detection marker stays in RViz.
MARKER_LIFETIME_SEC = 3.0

#: Marker colours per class family, RGB in 0..1. Matches the image overlay.
MARKER_COLORS = {
    "pedestrian": (1.0, 0.38, 0.13),
    "people": (1.0, 0.59, 0.16),
    "car": (0.16, 0.78, 1.0),
    "van": (0.24, 0.67, 1.0),
    "truck": (0.35, 0.55, 1.0),
    "bus": (0.47, 0.47, 1.0),
}
DEFAULT_MARKER_COLOR = (0.78, 0.78, 0.78)


class TargetLocalizerNode(Node):
    """Back-project detections onto the ground plane in the navigation frame."""

    def __init__(self) -> None:
        super().__init__("target_localizer")
        self.declare_parameter("detection_config", "")
        config_path = str(self.get_parameter("detection_config").value).strip()
        if not config_path:
            raise DetectionConfigError("parameter 'detection_config' is required")
        self._config = DetectionConfig.load(Path(config_path))

        self._poses = PoseBuffer(history_sec=self._config.localizer.odom_buffer_sec)
        topics = self._config.topics
        self._world_pub = self.create_publisher(
            Detection3DArray, topics.detections_world,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self._marker_pub = self.create_publisher(
            MarkerArray, topics.markers, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Detection2DArray, topics.detections, self._on_detections,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Odometry, topics.vehicle_odom, self._on_odom,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info(
            f"target_localizer: '{topics.detections}' -> '{topics.detections_world}' in frame "
            f"'{self._config.world.frame_id}', ground z={self._config.world.ground_z_m:.3f} m, "
            f"anchor '{self._config.localizer.anchor}'")

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._poses.add(TimedPose(
            stamp_sec=time_seconds(message.header.stamp),
            position=(position.x, position.y, position.z),
            orientation=(orientation.w, orientation.x, orientation.y, orientation.z),
        ))

    def _on_detections(self, message: Detection2DArray) -> None:
        stamp_sec = time_seconds(message.header.stamp)
        pose = self._poses.nearest(
            stamp_sec, max_age_sec=self._config.localizer.max_pose_age_sec)
        if pose is None:
            self.get_logger().warn(
                "no vehicle pose within "
                f"{self._config.localizer.max_pose_age_sec:.2f} s of the detection stamp; "
                "dropping this frame", throttle_duration_sec=5.0)
            return

        localized = []
        for detection in message.detections:
            box = pixel_box(detection.bbox)
            placed = ground_position(
                self._config.camera, self._config.camera_extrinsics, pose, box,
                anchor=self._config.localizer.anchor,
                ground_z_m=self._config.world.ground_z_m,
                max_range_m=self._config.localizer.max_range_m)
            if placed is None:
                continue
            position, _ = placed
            footprint = ground_footprint(
                self._config.camera, self._config.camera_extrinsics, pose, box,
                ground_z_m=self._config.world.ground_z_m,
                max_range_m=self._config.localizer.max_range_m) or (0.0, 0.0)
            localized.append(detection_3d(
                detection,
                position=position,
                size=footprint,
                # A single image box carries no height information; a consumer
                # that needs one must get it from the class, not from here.
                height=0.0,
                stamp=message.header.stamp,
                frame_id=self._config.world.frame_id,
            ))

        self._world_pub.publish(detection_3d_array(
            localized, stamp=message.header.stamp, frame_id=self._config.world.frame_id))
        if localized:
            self._marker_pub.publish(self._markers(localized, message.header.stamp))

    def _markers(self, detections, stamp) -> MarkerArray:
        markers = MarkerArray()
        for index, detection in enumerate(detections):
            class_id, score = hypothesis_of(detection)
            red, green, blue = MARKER_COLORS.get(class_id, DEFAULT_MARKER_COLOR)
            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = self._config.world.frame_id
            body.ns = "detections"
            body.id = index
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose = detection.bbox.center
            body.pose.position.z += 0.5
            body.scale.x = max(detection.bbox.size.x, 1.0)
            body.scale.y = max(detection.bbox.size.y, 1.0)
            body.scale.z = 1.0
            body.color.r, body.color.g, body.color.b = red, green, blue
            body.color.a = 0.6
            body.lifetime.sec = int(MARKER_LIFETIME_SEC)
            markers.markers.append(body)

            label = Marker()
            label.header = body.header
            label.ns = "detection_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = detection.bbox.center.position.x
            label.pose.position.y = detection.bbox.center.position.y
            label.pose.position.z = detection.bbox.center.position.z + 2.0
            label.pose.orientation.w = 1.0
            label.scale.z = 1.5
            label.color.r, label.color.g, label.color.b = red, green, blue
            label.color.a = 0.9
            label.text = f"{class_id} {score:.2f}"
            label.lifetime.sec = int(MARKER_LIFETIME_SEC)
            markers.markers.append(label)
        return markers


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        node = TargetLocalizerNode()
    except DetectionConfigError as error:
        print(f"target_localizer: {error}")
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
