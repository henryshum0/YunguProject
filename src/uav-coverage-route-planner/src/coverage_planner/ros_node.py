"""ROS 2 adapter for one-shot coverage planning and visualization."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
from math import cos, isfinite, radians, sin

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from coverage_planner.io import load_config
from coverage_planner.models.config import StartupConfig
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.planner import PlanResult
from coverage_planner.runtime import plan_from_config


def latched_qos() -> QoSProfile:
    """QoS used for startup results consumed by RViz and late subscribers."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def quaternion_from_compass_heading(heading_deg: float) -> tuple[float, float, float, float]:
    """Convert clockwise-from-North heading to ROS counter-clockwise yaw."""
    yaw_rad = radians(90.0 - heading_deg)
    return 0.0, 0.0, sin(yaw_rad / 2.0), cos(yaw_rad / 2.0)


def build_path(
    route: Sequence[Waypoint], *, frame_id: str, stamp: Time,
) -> Path:
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for waypoint in route:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(waypoint.x)
        pose.pose.position.y = float(waypoint.y)
        pose.pose.position.z = float(waypoint.z)
        x, y, z, w = quaternion_from_compass_heading(waypoint.yaw_deg)
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        message.poses.append(pose)
    return message


def build_markers(
    config: StartupConfig,
    route: Sequence[Waypoint],
    *,
    stamp: Time,
) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(_line_strip_marker(
        config.search_area_points,
        frame_id=config.frame_id,
        stamp=stamp,
        namespace="search_area",
        marker_id=0,
        z=config.flight.ground_elevation_m,
        color=(0.0, 1.0, 0.0, 1.0),
    ))
    for marker_id, area in enumerate(config.occupied_areas):
        markers.markers.append(_line_strip_marker(
            area.points,
            frame_id=config.frame_id,
            stamp=stamp,
            namespace="occupied_areas",
            marker_id=marker_id,
            z=config.flight.ground_elevation_m,
            color=(1.0, 0.0, 0.0, 1.0),
        ))
    waypoints = Marker()
    waypoints.header.frame_id = config.frame_id
    waypoints.header.stamp = stamp
    waypoints.ns = "waypoints"
    waypoints.id = 0
    waypoints.type = Marker.POINTS
    waypoints.action = Marker.ADD
    waypoints.pose.orientation.w = 1.0
    waypoints.scale.x = 1.25
    waypoints.scale.y = 1.25
    waypoints.color.r = 0.0
    waypoints.color.g = 0.25
    waypoints.color.b = 1.0
    waypoints.color.a = 1.0
    waypoints.points = [
        Point(x=float(item.x), y=float(item.y), z=float(item.z)) for item in route
    ]
    markers.markers.append(waypoints)
    return markers


def _line_strip_marker(
    points: Sequence[tuple[float, float]],
    *,
    frame_id: str,
    stamp: Time,
    namespace: str,
    marker_id: int,
    z: float,
    color: tuple[float, float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    ring = (*points, points[0])
    marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y in ring]
    return marker


def search_area_from_polygon(
    message: PolygonStamped, *, expected_frame_id: str,
) -> tuple[tuple[float, float], ...]:
    """Validate the four ENU corners supplied by a planning trigger."""
    if message.header.frame_id != expected_frame_id:
        raise ValueError(
            f"search-area trigger frame_id must be '{expected_frame_id}', got "
            f"'{message.header.frame_id}'")
    vertices = message.polygon.points
    if len(vertices) != 4:
        raise ValueError(
            f"search-area trigger must contain exactly four points, got {len(vertices)}")
    points = tuple((float(vertex.x), float(vertex.y)) for vertex in vertices)
    if not all(isfinite(value) for point in points for value in point):
        raise ValueError("search-area trigger points must be finite ENU coordinates")
    if len(set(points)) != 4:
        raise ValueError("search-area trigger must contain four distinct points")
    from shapely.geometry import Polygon

    polygon = Polygon(points)
    if polygon.is_empty or polygon.area <= 0.0 or not polygon.is_valid:
        raise ValueError("search-area trigger must form a valid simple quadrilateral")
    return points


class CoveragePlannerNode(Node):
    """Plans from JSON at startup and accepts live four-corner service requests."""

    def __init__(self) -> None:
        super().__init__("coverage_planner")
        self.declare_parameter("config_file", "")
        self.config: StartupConfig | None = None
        self.result: PlanResult | None = None
        self.waypoint_publisher = None
        self.marker_publisher = None
        self.plan_service = None

    def plan_and_publish(self, config: StartupConfig | None = None) -> Path:
        """Plan from the supplied config (or the configured JSON) and publish it."""
        if config is None:
            config = self._load_config()
        result = plan_from_config(config)
        stamp = self.get_clock().now().to_msg()
        path = build_path(result.planning_route, frame_id=config.frame_id, stamp=stamp)
        markers = build_markers(config, result.planning_route, stamp=stamp)
        if self.waypoint_publisher is None:
            qos = latched_qos()
            self.waypoint_publisher = self.create_publisher(
                Path, config.output_topics.waypoints, qos)
            self.marker_publisher = self.create_publisher(
                MarkerArray, config.output_topics.markers, qos)
        self.waypoint_publisher.publish(path)
        self.marker_publisher.publish(markers)
        self.config = config
        self.result = result
        self.get_logger().info(
            f"published {len(path.poses)} sparse waypoints on "
            f"'{config.output_topics.waypoints}' in frame '{config.frame_id}'")
        return path

    def start(self) -> None:
        """Publish the configured area once and expose the replanning service."""
        from coverage_planner.srv import PlanCoverage

        self.plan_and_publish()
        self.plan_service = self.create_service(
            PlanCoverage,
            "~/plan_coverage",
            self._handle_plan_coverage,
        )
        self.get_logger().info(
            "waiting for four-corner coverage requests on "
            "'/coverage_planner/plan_coverage'")

    def _load_config(self) -> StartupConfig:
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            raise ValueError(
                "required ROS parameter 'config_file' is empty; pass "
                "--ros-args -p config_file:=/absolute/path/to/config.json")
        return load_config(config_file)

    def _handle_plan_coverage(self, request, response):
        """Plan the request's search area and return the sparse route to the caller."""
        if self.config is None:
            response.success = False
            response.message = "coverage planner startup has not completed"
            return response
        try:
            points = search_area_from_polygon(
                request.search_area, expected_frame_id=self.config.frame_id)
            path = self.plan_and_publish(replace(self.config, search_area_points=points))
            response.success = True
            response.message = f"planned {len(path.poses)} sparse waypoints"
            response.waypoints = path
        except Exception as exc:  # noqa: BLE001 - service errors must not terminate the node
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"coverage request rejected: {exc}")
        return response


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        node.start()
    except Exception as exc:  # noqa: BLE001 - fatal boundary must reject every startup failure
        node.get_logger().fatal(f"coverage planning startup failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
