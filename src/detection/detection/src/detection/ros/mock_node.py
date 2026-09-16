"""ROS node publishing the mock detector's output.

Replacing this node with a real detector is the whole point of the design: the
messages it publishes, and the topics it publishes them on, are the ones a real
detector node must produce. Nothing downstream knows which of the two is running.

The node does one thing and stays out of everything else's way: the debug
overlay lives in its own node so that decoding a camera stream cannot disturb the
detector's timing.

Two clock details matter here:

* The vehicle pose comes from ``/gz/odom_super``, which is already the ENU pose
  in the launch-origin navigation frame and is already stamped with the wall
  clock the navigation stack uses. Detections are stamped with the pose sample
  they were generated from, so detection time and flight time are the same clock.
* Inference latency is reproduced by holding a finished frame back: the message
  keeps its capture stamp but is published later, exactly as a real pipeline
  delivers a result about the past.
"""

from __future__ import annotations

from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray

from detection.config import (
    DetectionConfig,
    DetectionConfigError,
    MockDetectorConfig,
    load_targets,
    resolve_workspace_path,
)
from detection.error_model import DetectionErrorModel
from detection.mock import MockDetector, MockFrame, VehiclePose
from detection.occlusion import OcclusionMesh, OcclusionMeshError
from detection.ros.conversions import detection_2d_array, time_seconds


#: How often finished frames are checked for release, in Hz. Must be well above
#: the detector rate so latency is reproduced with millisecond resolution.
RELEASE_RATE_HZ = 100.0

#: Seconds between the periodic status log.
STATUS_PERIOD_SEC = 10.0


class MockDetectorNode(Node):
    """Publish synthetic detections for the configured ground-truth targets."""

    def __init__(self) -> None:
        super().__init__("mock_detector")
        self.declare_parameter("detection_config", "")
        self.declare_parameter("mock_config", "")
        self.declare_parameter("targets_config", "")
        self.declare_parameter("collision_mesh", "")

        detection_path = self._required_path("detection_config")
        mock_path = self._required_path("mock_config")
        targets_path = self._required_path("targets_config")

        self._detection = DetectionConfig.load(detection_path)
        self._mock = MockDetectorConfig.load(mock_path)
        targets = load_targets(targets_path, ground_z_m=self._detection.world.ground_z_m)
        if not targets.targets:
            self.get_logger().warn(
                f"no ground-truth targets declared in '{targets_path}'; "
                "the mock detector will only publish false positives")

        mesh = self._load_occlusion_mesh(mock_path)
        self._detector = MockDetector(
            camera=self._detection.camera,
            camera_extrinsics=self._detection.camera_extrinsics,
            targets=targets,
            error_model=DetectionErrorModel(self._mock.error_model, seed=self._mock.seed),
            visibility=self._mock.visibility,
            occlusion_mesh=mesh,
        )

        topics = self._detection.topics
        self._detections_pub = self.create_publisher(
            Detection2DArray, topics.detections,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self._odom_sub = self.create_subscription(
            Odometry, topics.vehicle_odom, self._on_odom,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        self._pose: VehiclePose | None = None
        self._pending: list[MockFrame] = []
        self._frame_index = 0
        self._published_frames = 0
        self._published_detections = 0
        self._warned_missing_pose = False

        self.create_timer(1.0 / self._mock.rate_hz, self._on_capture)
        self.create_timer(1.0 / RELEASE_RATE_HZ, self._on_release)
        self.create_timer(STATUS_PERIOD_SEC, self._on_status)

        self.get_logger().info(
            f"mock_detector: {len(targets.targets)} target(s), {self._mock.rate_hz:.1f} Hz, "
            f"camera {self._detection.camera.width}x{self._detection.camera.height} "
            f"fx={self._detection.camera.fx:.1f}, occlusion "
            f"{'on' if self._detector.occlusion_enabled else 'off'}, "
            f"pose from '{topics.vehicle_odom}' -> '{topics.detections}'")

    # -- configuration ------------------------------------------------------
    def _required_path(self, name: str) -> Path:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise DetectionConfigError(f"parameter '{name}' is required")
        return Path(value)

    def _load_occlusion_mesh(self, mock_path: Path) -> OcclusionMesh | None:
        occlusion = self._mock.visibility.occlusion
        if not occlusion.enabled:
            return None
        override = str(self.get_parameter("collision_mesh").value).strip()
        mesh_path = resolve_workspace_path(override or occlusion.collision_mesh, reference=mock_path)
        try:
            mesh = OcclusionMesh.from_binary_stl(
                mesh_path, origin_offset=occlusion.world_origin_in_gz)
        except OcclusionMeshError as error:
            self.get_logger().warn(
                f"occlusion disabled: {error}. Targets behind buildings will be reported.")
            return None
        self.get_logger().info(f"occlusion mesh '{mesh_path}': {len(mesh)} triangles")
        return mesh

    # -- inputs -------------------------------------------------------------
    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._pose = VehiclePose(
            position=(position.x, position.y, position.z),
            orientation=(orientation.w, orientation.x, orientation.y, orientation.z),
            stamp_sec=time_seconds(message.header.stamp),
        )

    # -- detector loop ------------------------------------------------------
    def _on_capture(self) -> None:
        pose = self._pose
        if pose is None:
            if not self._warned_missing_pose:
                self.get_logger().warn(
                    f"waiting for vehicle odometry on "
                    f"'{self._detection.topics.vehicle_odom}'; no detections until it arrives")
                self._warned_missing_pose = True
            return
        self._warned_missing_pose = False
        self._frame_index += 1
        self._pending.append(self._detector.detect(pose, frame_index=self._frame_index))

    def _on_release(self) -> None:
        """Publish every frame whose simulated inference latency has elapsed."""
        if not self._pending:
            return
        now = self._now_sec()
        ready = [frame for frame in self._pending if frame.publish_at_sec <= now]
        if not ready:
            return
        self._pending = [frame for frame in self._pending if frame.publish_at_sec > now]
        for frame in ready:
            self._detections_pub.publish(detection_2d_array(
                frame.detections,
                stamp_sec=frame.stamp_sec,
                frame_id=self._detection.camera.frame_id,
            ))
            self._published_frames += 1
            self._published_detections += len(frame.detections)

    def _on_status(self) -> None:
        if self._published_frames == 0:
            return
        self.get_logger().info(
            f"mock_detector: {self._published_frames} frames, "
            f"{self._published_detections} detections published")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        node = MockDetectorNode()
    except DetectionConfigError as error:
        print(f"mock_detector: {error}")
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
