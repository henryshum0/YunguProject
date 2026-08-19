#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Populate the offboard node's waypoint buffer with a serpentine
(boustrophedon) path over the benchmark map.

This is a standalone node (run independently from the offboard stack):

    ros2 run benchmark waypoint_populator [options]

The path sweeps the field from TOP to BOTTOM, alternating LEFT-to-RIGHT /
RIGHT-to-LEFT between consecutive rows (a lawnmower pattern). Each waypoint is
published one at a time to the offboard node's waypoint buffer topic
(``/waypoint_buffer`` by default, reliable QoS), so the offboard node buffers
and flies them in order. Pointing ``--topic /waypoint_pose`` instead routes the
waypoints through the goal marker node (same as RViz 2D Goal Pose clicks).

Options:
  --config PATH     benchmark.yaml (default: <pkg>/config/benchmark.yaml)
  --topic NAME      buffer/ingestion topic to publish to
                    (default: /waypoint_buffer)
  --frame-id NAME   waypoint frame (default: world)
  --map-x M         field width  [m] (default: from benchmark.yaml)
  --map-y M         field height [m] (default: from benchmark.yaml)
  --spacing M       row spacing [m] (default: 5.0)
  --margin M        inset from the field edges [m] (default: 1.0)
  --z M             waypoint altitude [m] (default: 5.0)
  --start-delay S   seconds to wait after start before publishing (default: 5.0)
  --interval S      seconds between waypoint publishes (default: 0.05)
"""

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

try:
    from .generate_map import find_project_root, load_config
except ImportError:  # run directly as a plain script
    from generate_map import find_project_root, load_config


def serpentine(map_x: float, map_y: float, spacing: float, margin: float):
    """Return a list of (x, y) waypoints sweeping the field top-to-bottom,
    alternating left-to-right / right-to-left between rows."""
    x0, x1 = margin, map_x - margin
    y0, y1 = margin, map_y - margin   # y0 = bottom, y1 = top
    if x0 >= x1 or y0 >= y1 or spacing <= 0.0:
        return []
    n_rows = int((y1 - y0) / spacing) + 1
    pts = []
    left_to_right = True
    y = y1
    for _ in range(n_rows):
        if left_to_right:
            pts.append((x0, y))
            pts.append((x1, y))
        else:
            pts.append((x1, y))
            pts.append((x0, y))
        left_to_right = not left_to_right
        y -= spacing
    return pts


def _default_config_path():
    root = find_project_root(Path(__file__).resolve())
    if root is None:
        root = Path(__file__).resolve().parents[2]
    cand = root / 'src' / 'benchmark' / 'config' / 'benchmark.yaml'
    return str(cand) if cand.is_file() else None


class WaypointPopulator(Node):
    def __init__(self, topic, frame_id, map_x, map_y, spacing, margin, z,
                 start_delay, interval):
        super().__init__('waypoint_populator')
        self._topic = topic
        self._frame_id = frame_id
        self._z = float(z)
        self._interval = float(interval)
        self._pts = serpentine(float(map_x), float(map_y),
                               float(spacing), float(margin))
        self._idx = 0
        self._started = False
        self._timer = None

        # Reliable/volatile so the offboard node's reliable /waypoint_buffer
        # subscription never drops a waypoint.
        self.pub_ = self.create_publisher(
            PoseStamped, self._topic,
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

        self.get_logger().info(
            f"Serpentine waypoint populator: {len(self._pts)} waypoints over "
            f"{float(map_x):.1f} x {float(map_y):.1f} m (spacing "
            f"{float(spacing):.2f}, margin {float(margin):.1f}, z {self._z:.1f}) "
            f"-> {self._topic}")

        if not self._pts:
            self.get_logger().error(
                "Invalid field geometry (margin >= map/2 or spacing <= 0); "
                "no waypoints generated")
            return

        self._timer = self.create_timer(float(start_delay), self._on_tick)

    def _on_tick(self):
        if not self._started:
            # First tick: switch from the start-delay timer to the per-waypoint
            # interval timer.
            self._started = True
            if self._timer is not None:
                self._timer.cancel()
            if self.pub_.get_subscription_count() == 0:
                # Nobody is listening yet (the offboard node may still be
                # coming up / being discovered). Wait for a subscriber so no
                # waypoint is dropped.
                self.get_logger().info(
                    f"No subscriber on {self._topic} yet - waiting for the "
                    "offboard node to connect ...")
                self._timer = self.create_timer(0.5, self._check_subscribers)
                return
            self._begin_publish()
            return

        if self._idx >= len(self._pts):
            self._finish()
            return
        self._publish_one()

    def _check_subscribers(self):
        if self.pub_.get_subscription_count() > 0:
            self.get_logger().info("Subscriber connected; starting.")
            if self._timer is not None:
                self._timer.cancel()
            self._begin_publish()

    def _begin_publish(self):
        self.get_logger().info(
            f"Publishing {len(self._pts)} waypoints to {self._topic} ...")
        if self._interval <= 0.0:
            while self._idx < len(self._pts):
                self._publish_one()
            self._finish()
            return
        self._timer = self.create_timer(self._interval, self._on_tick)

    def _publish_one(self):
        x, y = self._pts[self._idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = self._z
        msg.pose.orientation.w = 1.0
        self.pub_.publish(msg)
        self.get_logger().info(
            f"Waypoint {self._idx + 1}/{len(self._pts)}: "
            f"({x:.2f}, {y:.2f}, {self._z:.1f})")
        self._idx += 1

    def _finish(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info(
            f"All {len(self._pts)} waypoints published to {self._topic}; "
            "the offboard node's waypoint buffer is now populated.")


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Populate the offboard waypoint buffer with a serpentine '
                    'path over the benchmark map.')
    parser.add_argument('--config', default=None,
                        help='Path to benchmark.yaml '
                             '(default: <pkg>/config/benchmark.yaml)')
    parser.add_argument('--topic', default='/waypoint_buffer',
                        help='Buffer/ingestion topic to publish to '
                             '(default: /waypoint_buffer; /waypoint_pose also '
                             'works and routes through the goal marker node)')
    parser.add_argument('--frame-id', default='world',
                        help='Fixed frame of the waypoints (default: world)')
    parser.add_argument('--map-x', type=float, default=None,
                        help='Field width [m] (default: from benchmark.yaml)')
    parser.add_argument('--map-y', type=float, default=None,
                        help='Field height [m] (default: from benchmark.yaml)')
    parser.add_argument('--spacing', type=float, default=5.0,
                        help='Row spacing [m] (default: 5.0)')
    parser.add_argument('--margin', type=float, default=1.0,
                        help='Inset from the field edges [m] (default: 1.0)')
    parser.add_argument('--z', type=float, default=5.0,
                        help='Waypoint altitude [m] (default: 5.0)')
    parser.add_argument('--start-delay', type=float, default=5.0,
                        help='Seconds to wait after start before publishing '
                             '(default: 5.0)')
    parser.add_argument('--interval', type=float, default=0.05,
                        help='Seconds between waypoint publishes '
                             '(default: 0.05)')
    opts = parser.parse_args(args)

    # Map size: CLI flags win; otherwise read from benchmark.yaml.
    map_x = opts.map_x
    map_y = opts.map_y
    if map_x is None or map_y is None:
        config_path = Path(opts.config) if opts.config else \
            Path(_default_config_path())
        if config_path is not None and config_path.is_file():
            cfg = load_config(config_path)
            if map_x is None:
                map_x = float(cfg['map']['x'])
            if map_y is None:
                map_y = float(cfg['map']['y'])
    if map_x is None or map_y is None:
        sys.exit('ERROR: could not determine map size; pass --map-x/--map-y '
                 'or a valid --config')
    if map_x <= 0.0 or map_y <= 0.0:
        sys.exit('ERROR: map size must be positive')

    rclpy.init(args=[])
    node = WaypointPopulator(opts.topic, opts.frame_id, map_x, map_y,
                             opts.spacing, opts.margin, opts.z,
                             opts.start_delay, opts.interval)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
