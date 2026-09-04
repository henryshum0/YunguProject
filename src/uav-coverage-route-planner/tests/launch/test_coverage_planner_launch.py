from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path as FilePath

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray
from coverage_planner.srv import PlanCoverage


TEST_NODE_NAME = "coverage_planner_launch_test_node"
TEST_WAYPOINTS_TOPIC = "/coverage_planner_launch_test/waypoints"
TEST_MARKERS_TOPIC = "/coverage_planner_launch_test/markers"


def _test_config() -> str:
    """Write unique output topics so a running stack cannot affect this test."""
    share = FilePath(get_package_share_directory("coverage_planner"))
    payload = json.loads((share / "config" / "example_planner.json").read_text())
    payload["map_file"] = str(share / "config" / "example_map.json")
    payload["output_topics"] = {
        "waypoints": TEST_WAYPOINTS_TOPIC,
        "markers": TEST_MARKERS_TOPIC,
    }
    directory = FilePath(tempfile.mkdtemp(prefix="coverage-planner-launch-test-"))
    config_file = directory / "planner.json"
    config_file.write_text(json.dumps(payload), encoding="utf-8")
    return str(config_file)


def generate_test_description():
    planner = launch_ros.actions.Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        name=TEST_NODE_NAME,
        parameters=[{"config_file": _test_config()}],
        output="screen",
    )
    return launch.LaunchDescription([
        planner,
        launch_testing.actions.ReadyToTest(),
    ]), {"planner": planner}


class TestCoveragePlannerTopics(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_publishes_path_and_markers_only_after_service_request(self) -> None:
        node = rclpy.create_node("coverage_planner_launch_test")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        received: dict[str, object] = {}
        node.create_subscription(
            Path,
            TEST_WAYPOINTS_TOPIC,
            lambda message: received.setdefault("path", message),
            qos,
        )
        node.create_subscription(
            MarkerArray,
            TEST_MARKERS_TOPIC,
            lambda message: received.setdefault("markers", message),
            qos,
        )
        deadline = time.monotonic() + 30.0
        try:
            startup_deadline = time.monotonic() + 2.0
            while time.monotonic() < startup_deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertEqual(received, {})

            client = node.create_client(
                PlanCoverage, f"/{TEST_NODE_NAME}/plan_coverage")
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
            request = PlanCoverage.Request()
            request.search_area.header.frame_id = "map"
            request.search_area.polygon.points = [
                Point32(x=10.0, y=10.0), Point32(x=80.0, y=10.0),
                Point32(x=80.0, y=60.0), Point32(x=10.0, y=60.0),
            ]
            request.publish_result = False
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(future.done())
            response = future.result()
            self.assertTrue(response.success, response.message)
            self.assertGreater(len(response.waypoints.poses), 2)
            self.assertEqual(response.waypoints.header.frame_id, "map")

            preview_deadline = time.monotonic() + 2.0
            while time.monotonic() < preview_deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertEqual(received, {})

            request.publish_result = True
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(future.done())
            response = future.result()
            self.assertTrue(response.success, response.message)

            while len(received) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertIn("path", received)
            self.assertIn("markers", received)
            path = received["path"]
            markers = received["markers"]
            self.assertGreater(len(path.poses), 2)
            self.assertEqual(path.header.frame_id, "map")
            self.assertEqual(path.poses[0].pose.position.x, path.poses[-1].pose.position.x)
            self.assertGreaterEqual(len(markers.markers), 2)
        finally:
            node.destroy_node()
