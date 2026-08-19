#!/usr/bin/env python3
"""csp_adapter — feed a coverage-search-planner flight_plan.json into the
offboard waypoint pipeline.

Reads    flight_plan.json  (protocol v3.0, ENU meters)
Publishes  /waypoint_pose  (geometry_msgs/PoseStamped, world ENU)
Calls    /offboard/land   (std_srvs/Trigger) once the final waypoint is out.

Full integration design (coordinate calibration method A/B, protocol checks,
QoS constraints) lives in docs/coverage-search-integration.md at the repo
root. Design decisions repeated here:

- The /waypoint_pose subscriber (goal_marker_node) is best_effort +
  keep_last(1), so the publisher must be best_effort too (a reliable
  publisher would not match it), and waypoints must be spaced >= 0.5 s.
- offboard + SUPER fly their own trajectory; speed_mps from the plan is not
  consumed downstream. To enforce cruise speed, cap SUPER's velocity in its
  planner config instead of throttling here (throttling the publish rate
  does not pace flight: the offboard buffer is FIFO).
- turn_in_place waypoints pause here for hold_time_s before the next
  waypoint goes out (offboard's waypoint_hold_time is global, not per-point).

Usage (direct):
  python3 csp_adapter/csp_adapter.py --plan-path results/example_run/flight_plan.json \
      --offset-x 0.0 --offset-y 0.0 --offset-z 0.0

Usage (launch, params are equivalent):
  ros2 launch csp_adapter csp_adapter.launch.py \
      plan_path:=... offset_x:=... offset_y:=... offset_z:=...
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

# ---------------------------------------------------------------------------
# Coordinate transform (pure functions, unit-testable)
# ---------------------------------------------------------------------------

def to_world(x, y, z, yaw_deg, tx, ty, tz):
    """Map a planner-local ENU point into world ENU.

    p_world = Rz(yaw_deg) * p_planner + T,  with T = (tx, ty, tz).
    yaw_deg is the extra rotation needed when the planner map's y axis does
    not align with world (0.0 = identity, see docs section 3).
    """
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    return (
        ca * x - sa * y + tx,
        sa * x + ca * y + ty,
        z + tz,
    )


def yaw_to_quat(yaw_rad):
    """ENU yaw (0 = north / +y, 90 = east / +x) as a z-axis quaternion."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


# ---------------------------------------------------------------------------
# Plan loading / validation (docs section 2)
# ---------------------------------------------------------------------------

def load_plan(path):
    with open(path, encoding="utf-8") as f:
        plan = json.load(f)

    errors = []
    if plan.get("schema_version") != "3.0":
        errors.append(f"schema_version must be '3.0', got {plan.get('schema_version')!r}")
    if plan.get("coordinate_frame") != "ENU":
        errors.append(f"coordinate_frame must be 'ENU', got {plan.get('coordinate_frame')!r}")
    status = (plan.get("summary") or {}).get("mission_status")
    if status != "ready":
        errors.append(f"summary.mission_status must be 'ready', got {status!r}")

    # Segment connectivity: each segment starts where the previous ended,
    # first segment starts at the take-off waypoint.
    segments = plan.get("route_segments", [])
    for prev, seg in zip(segments, segments[1:]):
        if seg.get("start_waypoint_id") != prev.get("end_waypoint_id"):
            errors.append(
                f"segment {seg.get('segment_id')} starts at waypoint "
                f"{seg.get('start_waypoint_id')} but previous ended at "
                f"{prev.get('end_waypoint_id')}"
            )
    waypoints = plan.get("waypoints", [])
    if segments and waypoints:
        first = waypoints[0]
        if segments[0].get("start_waypoint_id") != first.get("id"):
            errors.append(
                f"first segment starts at waypoint {segments[0].get('start_waypoint_id')} "
                f"but the plan begins at waypoint {first.get('id')}"
            )

    if errors:
        raise ValueError("flight_plan.json validation failed:\n  - " + "\n  - ".join(errors))

    by_id = {w["id"]: w for w in waypoints}
    for seg in segments:
        start, end = seg.get("start_waypoint_id"), seg.get("end_waypoint_id")
        if start not in by_id or end not in by_id:
            raise ValueError(
                f"segment {seg.get('segment_id')} references unknown waypoints "
                f"{start}/{end}"
            )
    return plan


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class CspAdapter(Node):
    def __init__(self, plan_path, offset=(0.0, 0.0, 0.0), yaw_deg=0.0,
                 interval_sec=0.5, auto_land=True, land_delay_sec=5.0):
        super().__init__("csp_adapter")

        self._plan_path = plan_path
        self._offset = offset
        self._yaw_deg = yaw_deg
        self._interval = max(interval_sec, 0.5)   # QoS: >= 0.5 s spacing
        self._auto_land = auto_land
        self._land_delay = land_delay_sec

        # Must be best_effort to match goal_marker_node's subscription.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(PoseStamped, "/waypoint_pose", qos)

        self._land_client = self.create_client(Trigger, "/offboard/land")

        self._plan = load_plan(plan_path)
        self._waypoints = sorted(self._plan["waypoints"], key=lambda w: w["sequence"])
        if not self._waypoints:
            raise ValueError("plan contains no waypoints")

        # Warn about values outside this stack's reference envelope; the
        # adapter itself does not enforce them (docs section 6).
        for wp in self._waypoints:
            if wp.get("speed_mps", 0.0) > 4.0:
                self.get_logger().warn(
                    f"waypoint {wp['id']}: speed_mps={wp['speed_mps']} > 4 m/s "
                    f"reference cruise (not consumed downstream; cap in SUPER config)")
                break
            if wp.get("z", 0.0) > 20.0:
                self.get_logger().warn(
                    f"waypoint {wp['id']}: z={wp['z']} m > 20 m reference ceiling; "
                    f"adjust planner_config flight_altitude_m if SUPER cannot fly it")
                break

        self._idx = 0
        self._hold_until = None   # monotonic time when a turn_in_place hold ends
        self._finished = False
        self._land_at = None      # monotonic time when the land call fires
        self._landing_fired = False
        self._n_turn = sum(1 for w in self._waypoints if w.get("turn_in_place"))

        self.get_logger().info(
            f"plan {plan_path}: {len(self._waypoints)} waypoints, "
            f"{len(self._plan['route_segments'])} segments, "
            f"{self._n_turn} turn_in_place points, mission_status=ready")
        self.get_logger().info(
            f"transform: Rz({yaw_deg} deg) + T=({offset[0]}, {offset[1]}, {offset[2]})")
        self.create_timer(self._interval, self._tick)

    # -- publishing --------------------------------------------------------

    def _tick(self):
        now = time.monotonic()

        if self._hold_until is not None:
            if now < self._hold_until:
                return
            self._hold_until = None

        if self._idx >= len(self._waypoints):
            self._schedule_land(now)
            return

        wp = self._waypoints[self._idx]
        x, y, z = to_world(wp["x"], wp["y"], wp["z"], self._yaw_deg,
                           *self._offset)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quat(math.radians(wp["heading_deg"]))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub.publish(msg)
        self.get_logger().info(
            f"waypoint {self._idx + 1}/{len(self._waypoints)} "
            f"id={wp['id']} world=({x:.1f}, {y:.1f}, {z:.1f}) "
            f"heading={wp['heading_deg']} deg, speed={wp['speed_mps']} m/s")

        hold = wp.get("hold_time_s", 0.0) if wp.get("turn_in_place") else 0.0
        if hold > 0.0:
            self._hold_until = now + hold
            self.get_logger().info(
                f"turn_in_place at waypoint {wp['id']}: holding {hold} s")
        self._idx += 1

    # -- landing -----------------------------------------------------------

    def _schedule_land(self, now):
        if self._finished:
            return
        self._finished = True
        self.get_logger().info(f"all {len(self._waypoints)} waypoints published")
        if self._auto_land:
            self._land_at = now + self._land_delay
            self.get_logger().info(
                f"calling /offboard/land in {self._land_delay} s ...")

    def _try_land(self):
        if not self._land_client.service_is_ready():
            self.get_logger().warn("/offboard/land not available yet, retrying")
            return False
        req = Trigger.Request()
        future = self._land_client.call_async(req)
        future.add_done_callback(self._land_done)
        return True

    def _land_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("landing triggered")
            else:
                self.get_logger().error(f"landing refused: {resp.message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"landing call failed: {exc}")
        self._landing_fired = True

    # -- main loop hook for the land deadline ------------------------------

    def spin_once_for_land(self):
        """Call periodically from main(): fires the land call on deadline.

        Kept out of the publishing timer so an exhausted plan does not keep
        a fast timer alive; main() polls at 2 Hz instead.
        """
        if self._land_at is None or self._landing_fired:
            return
        if time.monotonic() >= self._land_at:
            self._land_at = None
            self._try_land()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Feed coverage-search-planner flight_plan.json into "
                    "/waypoint_pose (world ENU).")
    parser.add_argument("--plan-path", required=True,
                        help="path to flight_plan.json (v3.0, mission_status=ready)")
    parser.add_argument("--offset-x", type=float, default=0.0,
                        help="world translation T_x [m] (docs section 3)")
    parser.add_argument("--offset-y", type=float, default=0.0,
                        help="world translation T_y [m]")
    parser.add_argument("--offset-z", type=float, default=0.0,
                        help="world translation T_z [m]")
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="extra map->world rotation about z [deg]")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="waypoint publish interval [s], >= 0.5")
    parser.add_argument("--no-land", action="store_true",
                        help="do not call /offboard/land after the last waypoint")
    parser.add_argument("--land-delay", type=float, default=5.0,
                        help="delay before /offboard/land after the last waypoint [s]")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = CspAdapter(
        args.plan_path,
        offset=(args.offset_x, args.offset_y, args.offset_z),
        yaw_deg=args.yaw_deg,
        interval_sec=args.interval,
        auto_land=not args.no_land,
        land_delay_sec=args.land_delay,
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            node.spin_once_for_land()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
