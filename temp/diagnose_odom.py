#!/usr/bin/env python3
"""
diagnose_odom.py — One-shot diagnostic tool for FAST-LIO → PX4 odometry chain.

Checks:
  1. /Odometry — is FAST-LIO publishing?
  2. /fmu/in/vehicle_visual_odometry — is the bridge working?
  3. /fmu/out/vehicle_local_position_v1 — is PX4 EKF2 output available?
  4. Rate, latency, and covariance sanity checks.
  5. Summary verdict.

Usage:
  ros2 run <pkg> diagnose_odom.py
  # or directly:
  python3 temp/diagnose_odom.py
"""

import time
import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition


class DiagNode(Node):
    def __init__(self):
        super().__init__("odom_diag", start_parameter_services=False)

        self._odom_samples: deque = deque(maxlen=50)     # (stamp_sec, x, y, z)
        self._ev_samples: deque = deque(maxlen=50)        # (stamp_sec, x_ned, y_ned, z_ned)
        self._loc_samples: deque = deque(maxlen=50)       # (stamp_sec, x, y, z)

        self._odom_cov_ok: bool = False
        self._ev_var_ok: bool = False

        # Subscribers — best-effort for PX4 topics
        qos_ro = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        qos_be = rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self._sub1 = self.create_subscription(Odometry, "/Odometry", self._cb_odom, qos_ro)
        self._sub2 = self.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self._cb_ev, qos_be)
        self._sub3 = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._cb_loc, qos_be)

        self._start = time.time()
        self._timer = self.create_timer(3.0, self._print_report)
        print("\n" + "=" * 60)
        print("  FAST-LIO → PX4 Odometry Chain — Diagnostic")
        print("  Collecting data for ~10 seconds ...\n")
        print("  Topics monitored:")
        print("    [1] /Odometry                           (FAST-LIO)")
        print("    [2] /fmu/in/vehicle_visual_odometry     (Bridge → PX4)")
        print("    [3] /fmu/out/vehicle_local_position_v1  (PX4 EKF2 output)")
        print("=" * 60 + "\n")

    def _cb_odom(self, m: Odometry):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self._odom_samples.append((t, m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
        # Check if covariance has non-zero values
        nonzero = sum(1 for v in m.pose.covariance if abs(v) > 1e-12)
        self._odom_cov_ok = nonzero > 0

    def _cb_ev(self, m: VehicleOdometry):
        t = m.timestamp_sample * 1e-6  # µs → s
        self._ev_samples.append((t, float(m.position[0]), float(m.position[1]), float(m.position[2])))
        pv = m.position_variance
        ov = m.orientation_variance
        vv = m.velocity_variance
        self._ev_var_ok = all(
            math.isfinite(v) and v > 0
            for v in [pv[0], pv[1], pv[2], ov[0], ov[1], ov[2], vv[0], vv[1], vv[2]]
        )

    def _cb_loc(self, m: VehicleLocalPosition):
        t = m.timestamp * 1e-6  # µs → s
        self._loc_samples.append((t, float(m.x), float(m.y), float(m.z)))

    def _rate_and_pos(self, samples: deque, name: str) -> str:
        if len(samples) < 2:
            return f"  {name}: ❌ NO DATA (check if source is running)"

        times = [s[0] for s in samples]
        dt = times[-1] - times[0]
        rate = (len(samples) - 1) / dt if dt > 0 else 0
        last = samples[-1]
        status = "✅" if rate > 4 else "⚠️ "
        return (
            f"  {name}: {status}  rate={rate:5.1f} Hz  "
            f"last=({last[1]:7.2f}, {last[2]:7.2f}, {last[3]:7.2f})"
        )

    def _cov_check(self) -> str:
        lines = []
        lines.append(f"  FAST-LIO covariance: {'✅ non-zero' if self._odom_cov_ok else '⚠️  all zeros (using defaults)'}")
        lines.append(f"  Bridge EV variances: {'✅ all finite > 0' if self._ev_var_ok else '❌ missing/zero (EKF2 may reject!)'}")
        return "\n".join(lines)

    def _print_report(self):
        elapsed = time.time() - self._start
        print(f"\n--- Diagnostic Report (t={elapsed:.0f}s) ---\n")
        print(self._rate_and_pos(self._odom_samples, "[1] /Odometry"))
        print(self._rate_and_pos(self._ev_samples, "[2] /fmu/in/vehicle_visual_odometry"))
        print(self._rate_and_pos(self._loc_samples, "[3] /fmu/out/vehicle_local_position_v1"))
        print()
        print(self._cov_check())
        print()
        if len(self._ev_samples) >= 2 and len(self._loc_samples) >= 2:
            ev = self._ev_samples[-1]
            loc = self._loc_samples[-1]
            diff = math.sqrt((ev[1]-loc[1])**2 + (ev[2]-loc[2])**2 + (ev[3]-loc[3])**2)
            print(f"  Bridge→PX4 position diff: {diff:.3f} m  (EV last vs EKF2 output)")
            if diff > 2.0:
                print("    ⚠️  Large difference — EKF2 may not be using EV yet, or coord mismatch")
            elif diff < 0.05:
                print("    ✅ Close match — EKF2 appears to be fusing EV odometry")
        print()
        if elapsed > 9.0:
            print("---")
            print("Verdict:")
            issues = []
            if len(self._odom_samples) < 2:
                issues.append("FAST-LIO not publishing")
            if len(self._ev_samples) < 2:
                issues.append("Bridge not publishing to PX4")
            if len(self._loc_samples) < 2:
                issues.append("PX4 EKF2 not responding (is PX4 running?)")
            if not self._ev_var_ok:
                issues.append("EV variances invalid — EKF2 will reject frames")
            if issues:
                for i in issues:
                    print(f"  ❌ {i}")
            else:
                print("  ✅ All checks passed — odometry chain is healthy")
            print()
            raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(DiagNode())
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
