"""ROS 2 adapter for one-shot coverage planning and visualization."""

from __future__ import annotations

from collections.abc import Sequence
from math import cos, isfinite, radians, sin
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
from coverage_planner.runtime import plan_for_search_area


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
    search_area_points: Sequence[tuple[float, float]],
    route: Sequence[Waypoint],
    *,
    stamp: Time,
) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(_line_strip_marker(
        search_area_points,
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
    """Loads map settings at startup and plans only on action goals."""

    def __init__(self) -> None:
        super().__init__("coverage_planner")
        self.declare_parameter("config_file", "")
        self.config: StartupConfig | None = None
        self.result: PlanResult | None = None
        self.waypoint_publisher = None
        self.marker_publisher = None
        self.plan_action = None
        self._planning_lock = Lock()

    def plan_and_publish(self, search_area_points: tuple[tuple[float, float], ...]) -> Path:
        """Plan and publish the search boundary supplied by an action goal."""
        config = self._require_config()
        result, path = self._plan(search_area_points)
        markers = build_markers(
            config, search_area_points, result.planning_route, stamp=path.header.stamp)
        self._ensure_publishers(config)
        self.waypoint_publisher.publish(path)
        self.marker_publisher.publish(markers)
        self.get_logger().info(
            f"published {len(path.poses)} sparse waypoints on "
            f"'{config.output_topics.waypoints}' in frame '{config.frame_id}'")
        return path

    def _plan(
        self, search_area_points: tuple[tuple[float, float], ...],
    ) -> tuple[PlanResult, Path]:
        """Plan and build a response path without publishing ROS output topics."""
        config = self._require_config()
        result = plan_for_search_area(config, search_area_points)
        stamp = self.get_clock().now().to_msg()
        path = build_path(result.planning_route, frame_id=config.frame_id, stamp=stamp)
        self.result = result
        return result, path

    def start(self) -> None:
        """Validate startup JSON and expose asynchronous on-demand planning."""
        from coverage_planner.action import PlanCoverage

        self.config = self._load_config()
        self._ensure_publishers(self.config)
        self.plan_action = ActionServer(
            self,
            PlanCoverage,
            "~/plan_coverage",
            execute_callback=self._execute_plan_coverage,
            goal_callback=self._accept_plan_goal,
            cancel_callback=self._cancel_plan_goal,
        )
        self.get_logger().info(
            "ready for asynchronous four-corner coverage actions on "
            "'/coverage_planner/plan_coverage'; routes publish only when requested")

    def _ensure_publishers(self, config: StartupConfig) -> None:
        """Create output publishers once without publishing an initial result."""
        if self.waypoint_publisher is not None:
            return
        qos = latched_qos()
        self.waypoint_publisher = self.create_publisher(
            Path, config.output_topics.waypoints, qos)
        self.marker_publisher = self.create_publisher(
            MarkerArray, config.output_topics.markers, qos)

    def _load_config(self) -> StartupConfig:
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            raise ValueError(
                "required ROS parameter 'config_file' is empty; pass "
                "--ros-args -p config_file:=/absolute/path/to/config.json")
        return load_config(config_file)

    def _require_config(self) -> StartupConfig:
        if self.config is None:
            raise RuntimeError("coverage planner startup has not completed")
        return self.config

    def _accept_plan_goal(self, _request):
        """Acknowledge receipt promptly; validation failures use the result channel."""
        return GoalResponse.ACCEPT

    def _cancel_plan_goal(self, _goal_handle):
        """Accept cancellation before execution; planning itself is not interruptible."""
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(goal_handle, stage: str) -> None:
        from coverage_planner.action import PlanCoverage

        feedback = PlanCoverage.Feedback()
        feedback.stage = stage
        goal_handle.publish_feedback(feedback)

    def _execute_plan_coverage(self, goal_handle):
        """Return success or a descriptive planning failure through the action result."""
        from coverage_planner.action import PlanCoverage

        result = PlanCoverage.Result()
        if self.config is None:
            result.success = False
            result.message = "coverage planner startup has not completed"
            goal_handle.abort()
            return result
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "coverage request cancelled before planning started"
            goal_handle.canceled()
            return result
        if not self._planning_lock.acquire(blocking=False):
            result.success = False
            result.message = "coverage planner is busy with another request"
            goal_handle.abort()
            return result
        try:
            self._feedback(goal_handle, "validating")
            points = search_area_from_polygon(
                goal_handle.request.search_area, expected_frame_id=self.config.frame_id)
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = "coverage request cancelled before planning started"
                goal_handle.canceled()
                return result
            self._feedback(goal_handle, "planning")
            if goal_handle.request.publish_result:
                self._feedback(goal_handle, "publishing")
                path = self.plan_and_publish(points)
            else:
                _, path = self._plan(points)
            result.success = True
            action = "planned and published" if goal_handle.request.publish_result else "planned"
            result.message = f"{action} {len(path.poses)} sparse waypoints"
            result.waypoints = path
            self._feedback(goal_handle, "succeeded")
            goal_handle.succeed()
        except Exception as exc:  # noqa: BLE001 - service errors must not terminate the node
            result.success = False
            result.message = str(exc)
            self._feedback(goal_handle, "failed")
            goal_handle.abort()
            self.get_logger().error(f"coverage request rejected: {exc}")
        finally:
            self._planning_lock.release()
        return result


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
