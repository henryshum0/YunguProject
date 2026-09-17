#!/usr/bin/env python3
"""Convert EGO-Planner's B-spline messages into an RViz ``nav_msgs/Path``.

EGO publishes its local trajectory as ``traj_utils/msg/Bspline``. RViz cannot
display that custom message directly, so this node samples the same De Boor
curve EGO's trajectory server evaluates and republishes it as a latched Path.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from traj_utils.msg import Bspline


Point = tuple[float, float, float]


def _finite_point(point: object) -> Point:
    """Convert a ROS point-like value to a finite three-dimensional tuple."""

    values = (float(point.x), float(point.y), float(point.z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("B-spline control points must be finite")
    return values


def _validate_bspline(message: Bspline) -> tuple[list[Point], list[float], int, float, float]:
    """Validate EGO's B-spline layout and return its valid knot domain.

    EGO uses ``order`` as the polynomial degree. Its own valid time range is
    ``[knots[order], knots[number_of_control_points]]``.
    """

    degree = int(message.order)
    control_points = [_finite_point(point) for point in message.pos_pts]
    knots = [float(knot) for knot in message.knots]
    if degree < 1:
        raise ValueError(f"B-spline order must be at least one, received {degree}")
    if len(control_points) < degree + 1:
        raise ValueError(
            f"B-spline order {degree} requires at least {degree + 1} control points, "
            f"received {len(control_points)}"
        )
    expected_knots = len(control_points) + degree + 1
    if len(knots) != expected_knots:
        raise ValueError(
            f"B-spline has {len(knots)} knots; expected {expected_knots} for "
            f"{len(control_points)} control points and order {degree}"
        )
    if not all(math.isfinite(knot) for knot in knots):
        raise ValueError("B-spline knots must be finite")
    if any(right < left for left, right in zip(knots, knots[1:])):
        raise ValueError("B-spline knots must be non-decreasing")
    begin = knots[degree]
    end = knots[len(control_points)]
    if end < begin:
        raise ValueError("B-spline valid knot interval is inverted")
    return control_points, knots, degree, begin, end


def evaluate_de_boor(control_points: Sequence[Point], knots: Sequence[float], degree: int, u: float) -> Point:
    """Evaluate an EGO-compatible B-spline at knot coordinate ``u``."""

    begin = knots[degree]
    end = knots[len(control_points)]
    u = min(max(begin, u), end)

    # This intentionally matches EGO's ``while (u[k + 1] < u)`` span choice.
    span = degree
    while span + 1 < len(knots) and knots[span + 1] < u:
        span += 1
    if span >= len(control_points):
        span = len(control_points) - 1

    work = [list(control_points[span - degree + index]) for index in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            left = knots[index + span - degree]
            right = knots[index + 1 + span - level]
            denominator = right - left
            if denominator <= 0.0:
                raise ValueError("B-spline contains a zero-width knot interval")
            alpha = (u - left) / denominator
            previous, current = work[index - 1], work[index]
            work[index] = [
                (1.0 - alpha) * previous[axis] + alpha * current[axis]
                for axis in range(3)
            ]
    return tuple(work[degree])  # type: ignore[return-value]


def sample_bspline(message: Bspline, sample_interval_sec: float, max_samples: int) -> list[Point]:
    """Return regularly sampled EGO trajectory points, including both ends."""

    if not math.isfinite(sample_interval_sec) or sample_interval_sec <= 0.0:
        raise ValueError("sample interval must be finite and greater than zero")
    if max_samples < 2:
        raise ValueError("max_samples must be at least two")
    control_points, knots, degree, begin, end = _validate_bspline(message)
    if end == begin:
        return [evaluate_de_boor(control_points, knots, degree, begin)]
    samples = min(max_samples, max(2, math.ceil((end - begin) / sample_interval_sec) + 1))
    return [
        evaluate_de_boor(control_points, knots, degree, begin + (end - begin) * index / (samples - 1))
        for index in range(samples)
    ]


def path_length_m(points: Sequence[Point]) -> float:
    """Return the Euclidean length of a sampled path."""

    return sum(
        math.dist(before, after)
        for before, after in zip(points, points[1:])
    )


def _orientation(points: Sequence[Point], index: int) -> tuple[float, float]:
    """Return planar yaw quaternion z/w components along a sampled path."""

    if len(points) < 2:
        return 0.0, 1.0
    before = points[max(0, index - 1)]
    after = points[min(len(points) - 1, index + 1)]
    dx, dy = after[0] - before[0], after[1] - before[1]
    if math.hypot(dx, dy) < 1.0e-6:
        return 0.0, 1.0
    half_yaw = math.atan2(dy, dx) / 2.0
    return math.sin(half_yaw), math.cos(half_yaw)


class EgoTrajectoryPath(Node):
    """Publish EGO's current B-spline as a reliable transient-local Path."""

    def __init__(self) -> None:
        super().__init__("ego_trajectory_path")
        self.declare_parameter("input_topic", "/ego_planner/bspline")
        self.declare_parameter("output_topic", "/ego_planner/trajectory")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("sample_interval_sec", 0.05)
        self.declare_parameter("max_samples", 400)
        self.declare_parameter("min_path_length_m", 0.05)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._sample_interval_sec = float(self.get_parameter("sample_interval_sec").value)
        self._max_samples = int(self.get_parameter("max_samples").value)
        self._min_path_length_m = float(self.get_parameter("min_path_length_m").value)
        if not self._frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(self._min_path_length_m) or self._min_path_length_m < 0.0:
            raise ValueError("min_path_length_m must be finite and zero or greater")

        source_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Path, output_topic, latched_qos)
        self._subscription = self.create_subscription(Bspline, input_topic, self._on_bspline, source_qos)
        self.get_logger().info(f"EGO trajectory view: {input_topic} -> {output_topic} ({self._frame_id})")

    def _on_bspline(self, message: Bspline) -> None:
        try:
            points = sample_bspline(message, self._sample_interval_sec, self._max_samples)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring malformed EGO B-spline: {error}")
            return
        if path_length_m(points) < self._min_path_length_m:
            # EGO emits zero-length hold/emergency trajectories too. Keeping
            # the last moving path makes RViz useful immediately after a
            # safety stop, while callers can set this threshold to zero when
            # they explicitly need to show holds.
            return

        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.z, pose.pose.orientation.w = _orientation(points, index)
            path.poses.append(pose)
        self._publisher.publish(path)


def main() -> None:
    rclpy.init()
    node = EgoTrajectoryPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
