from __future__ import annotations

from math import sqrt
from pathlib import Path

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker

from coverage_planner.io import ConfigError, load_config
from coverage_planner.models import Waypoint
from coverage_planner.ros_node import (
    CoveragePlannerNode,
    build_markers,
    build_path,
    latched_qos,
    quaternion_from_compass_heading,
    search_area_from_polygon,
)
from coverage_planner.runtime import PlanningFailed

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT.parent / "config"


def route() -> tuple[Waypoint, ...]:
    return (
        Waypoint("home", 1, "transit", 1, 2, 25, 0, -90, False),
        Waypoint("lane", 2, "capture", 8, 9, 25, 90, -90, True),
        Waypoint("return", 3, "transit", 1, 2, 25, 180, -90, False),
    )


def test_path_preserves_sparse_order_altitude_and_heading() -> None:
    message = build_path(route(), frame_id="map", stamp=Time(sec=12))
    assert message.header.frame_id == "map"
    assert [pose.pose.position.x for pose in message.poses] == [1.0, 8.0, 1.0]
    assert [pose.pose.position.z for pose in message.poses] == [25.0, 25.0, 25.0]
    assert message.poses[0].pose.orientation.z == pytest.approx(sqrt(0.5))
    assert message.poses[0].pose.orientation.w == pytest.approx(sqrt(0.5))
    assert message.poses[1].pose.orientation.z == pytest.approx(0.0)
    assert message.poses[1].pose.orientation.w == pytest.approx(1.0)
    assert quaternion_from_compass_heading(90) == pytest.approx((0, 0, 0, 1))


def test_markers_show_boundaries_obstacles_and_waypoint_points() -> None:
    config = load_config(CONFIG_DIR / "example_planner.json")
    message = build_markers(
        config,
        ((10.0, 10.0), (80.0, 10.0), (80.0, 60.0), (10.0, 60.0)),
        route(),
        stamp=Time(),
    )
    assert len(message.markers) == 3
    search, obstacle, waypoints = message.markers
    assert search.type == Marker.LINE_STRIP
    assert search.color.g == 1.0
    assert search.points[0] == search.points[-1]
    assert obstacle.type == Marker.LINE_STRIP
    assert obstacle.color.r == 1.0
    assert obstacle.points[0] == obstacle.points[-1]
    assert waypoints.type == Marker.POINTS
    assert waypoints.color.b == 1.0
    assert [(point.x, point.y, point.z) for point in waypoints.points] == [
        (1.0, 2.0, 25.0), (8.0, 9.0, 25.0), (1.0, 2.0, 25.0),
    ]


def test_qos_is_reliable_transient_local_keep_last_one() -> None:
    qos = latched_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_search_area_service_input_requires_four_map_frame_corners() -> None:
    message = PolygonStamped()
    message.header.frame_id = "map"
    message.polygon.points = [
        Point32(x=0.0, y=0.0), Point32(x=10.0, y=0.0),
        Point32(x=10.0, y=5.0), Point32(x=0.0, y=5.0),
    ]
    assert search_area_from_polygon(message, expected_frame_id="map") == (
        (0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0))
    message.header.frame_id = "odom"
    with pytest.raises(ValueError, match="frame_id"):
        search_area_from_polygon(message, expected_frame_id="map")
    message.header.frame_id = "map"
    message.polygon.points.pop()
    with pytest.raises(ValueError, match="exactly four"):
        search_area_from_polygon(message, expected_frame_id="map")


def test_invalid_config_creates_no_publishers(tmp_path: Path, monkeypatch) -> None:
    path = tmp_path / "invalid.json"
    path.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    rclpy.init(args=[])
    node = CoveragePlannerNode()
    try:
        node.set_parameters([Parameter("config_file", value=str(path))])
        with pytest.raises(ConfigError):
            node._load_config()
        assert node.waypoint_publisher is None
        assert node.marker_publisher is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_coverage_failure_creates_no_publishers(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))

    def fail_planning(config, points):
        raise PlanningFailed("required coverage was not achieved; failed patch IDs: patch_007")

    monkeypatch.setattr("coverage_planner.ros_node.plan_for_search_area", fail_planning)
    rclpy.init(args=[])
    node = CoveragePlannerNode()
    try:
        node.set_parameters([Parameter(
            "config_file", value=str(CONFIG_DIR / "example_planner.json"))])
        node.config = load_config(CONFIG_DIR / "example_planner.json")
        with pytest.raises(PlanningFailed, match="patch_007"):
            node.plan_and_publish(((10.0, 10.0), (80.0, 10.0), (80.0, 60.0), (10.0, 60.0)))
        assert node.waypoint_publisher is None
        assert node.marker_publisher is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
