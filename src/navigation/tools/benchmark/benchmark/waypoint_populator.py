#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Populate the offboard node's waypoint buffer with a serpentine
(boustrophedon) path over the benchmark map.

This is a standalone node (run independently from the offboard stack):

    ros2 run benchmark waypoint_populator [options]

The path sweeps the field from TOP to BOTTOM, alternating LEFT-to-RIGHT /
RIGHT-to-LEFT between consecutive rows (a lawnmower pattern). Each waypoint is
submitted in one request to the offboard node's waypoint-buffer service
(``/waypoint_buffer`` by default), so the offboard node queues and flies them
in order.

Options:
  --config PATH     benchmark.yaml (default: <pkg>/config/benchmark.yaml)
  --queue-service NAME
                    waypoint-buffer service to enqueue the route
                    (default: /waypoint_buffer)
  --frame-id NAME   waypoint frame (default: world)
  --map-x M         field width  [m] (default: from benchmark.yaml)
  --map-y M         field height [m] (default: from benchmark.yaml)
  --spacing M       row spacing [m] (default: 5.0)
  --margin M        inset from the field edges [m] (default: 1.0)
  --z M             waypoint altitude [m] (default: 5.0)
  --start-delay S   seconds to wait after start before publishing (default: 5.0)
"""

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from offboard_fsm.srv import QueueWaypoints

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
        root = Path(__file__).resolve().parents[5]
    cand = root / 'src' / 'navigation' / 'tools' / 'benchmark' / 'config' / 'benchmark.yaml'
    return str(cand) if cand.is_file() else None


class WaypointPopulator(Node):
    def __init__(self, queue_service, frame_id, map_x, map_y, spacing, margin, z,
                 start_delay):
        super().__init__('waypoint_populator')
        self._queue_service = queue_service
        self._frame_id = frame_id
        self._z = float(z)
        self._pts = serpentine(float(map_x), float(map_y),
                               float(spacing), float(margin))
        self._submitted = False
        self._timer = None

        self.queue_client = self.create_client(QueueWaypoints, self._queue_service)

        self.get_logger().info(
            f"Serpentine waypoint populator: {len(self._pts)} waypoints over "
            f"{float(map_x):.1f} x {float(map_y):.1f} m (spacing "
            f"{float(spacing):.2f}, margin {float(margin):.1f}, z {self._z:.1f}) "
            f"-> service {self._queue_service}")

        if not self._pts:
            self.get_logger().error(
                "Invalid field geometry (margin >= map/2 or spacing <= 0); "
                "no waypoints generated")
            return

        self._timer = self.create_timer(float(start_delay), self._on_tick)

    def _on_tick(self):
        if self._submitted:
            return
        if not self.queue_client.service_is_ready():
            self.get_logger().info(
                f"Waypoint queue service {self._queue_service} is not ready; waiting ...")
            return
        request = QueueWaypoints.Request()
        stamp = self.get_clock().now().to_msg()
        for x, y in self._pts:
            waypoint = PoseStamped()
            waypoint.header.stamp = stamp
            waypoint.header.frame_id = self._frame_id
            waypoint.pose.position.x = float(x)
            waypoint.pose.position.y = float(y)
            waypoint.pose.position.z = self._z
            waypoint.pose.orientation.w = 1.0
            request.waypoints.append(waypoint)
        self._submitted = True
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.queue_client.call_async(request).add_done_callback(self._queue_response)

    def _queue_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Waypoint queue request failed: {error}")
            return
        if not response.success or response.queued_count != len(self._pts):
            self.get_logger().error(
                f"Waypoint queue rejected the route: {response.message} "
                f"({response.queued_count}/{len(self._pts)} accepted)")
            return
        self.get_logger().info(
            f"Queued all {response.queued_count} waypoints on {self._queue_service}.")


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Populate the offboard waypoint buffer with a serpentine '
                    'path over the benchmark map.')
    parser.add_argument('--config', default=None,
                        help='Path to benchmark.yaml '
                             '(default: <pkg>/config/benchmark.yaml)')
    parser.add_argument('--queue-service', default='/waypoint_buffer',
                        help='Waypoint queue service (default: /waypoint_buffer)')
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
                        help='Seconds to wait after start before queueing '
                             '(default: 5.0)')
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
    node = WaypointPopulator(opts.queue_service, opts.frame_id, map_x, map_y,
                             opts.spacing, opts.margin, opts.z, opts.start_delay)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
