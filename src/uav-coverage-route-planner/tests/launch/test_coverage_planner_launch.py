from __future__ import annotations

import time
import unittest

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


def generate_test_description():
    share = get_package_share_directory("coverage_planner")
    planner = launch_ros.actions.Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        parameters=[{"config_file": f"{share}/config/example_planner.json"}],
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

    def test_receives_latched_path_and_markers(self) -> None:
        node = rclpy.create_node("coverage_planner_launch_test")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        received: dict[str, object] = {}
        node.create_subscription(
            Path,
            "/coverage_planner/waypoints",
            lambda message: received.setdefault("path", message),
            qos,
        )
        node.create_subscription(
            MarkerArray,
            "/coverage_planner/markers",
            lambda message: received.setdefault("markers", message),
            qos,
        )
        deadline = time.monotonic() + 30.0
        try:
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

            client = node.create_client(PlanCoverage, "/coverage_planner/plan_coverage")
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
            request = PlanCoverage.Request()
            request.search_area.header.frame_id = "map"
            request.search_area.polygon.points = [
                Point32(x=10.0, y=10.0), Point32(x=80.0, y=10.0),
                Point32(x=80.0, y=60.0), Point32(x=10.0, y=60.0),
            ]
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(future.done())
            response = future.result()
            self.assertTrue(response.success, response.message)
            self.assertGreater(len(response.waypoints.poses), 2)
            self.assertEqual(response.waypoints.header.frame_id, "map")
        finally:
            node.destroy_node()
