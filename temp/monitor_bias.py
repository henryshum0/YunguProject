#!/usr/bin/env python3
"""monitor_bias.py — Continuous flight monitor: FAST-LIO vs Gazebo truth bias.

Records the full flight at ~20 Hz into a CSV. Each topic's LATEST sample is
used (the FL/GT timestamp skew is only tens of ms, so no windowed alignment
is needed).

CSV columns (all sim-time, seconds):
  t, fl_x, fl_y, fl_z, fl_v, gt_x, gt_y, gt_z, gt_v,
  fus_x, fus_y, fus_z, dx, dy, dz, dxy, d3d
  dx = fl_x - gt_x (etc.), dxy = horizontal 2D error, d3d = 3D error

Usage:
  python3 temp/monitor_bias.py [out.csv]          # until Ctrl+C
  python3 temp/monitor_bias.py /tmp/flight1.csv 120   # fixed duration

Stop with Ctrl+C — the CSV is flushed on exit.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


def t_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class MonitorBias(Node):
    def __init__(self, csv_path):
        super().__init__("monitor_bias")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._latest = {"fl": None, "gt": None, "fus": None}

        self.create_subscription(Odometry, "/Odometry", self._mk_cb("fl"), qos)
        self.create_subscription(Odometry, "/odom", self._mk_cb("gt"), qos)
        self.create_subscription(Odometry, "/lidar_slam/odom", self._mk_cb("fus"), qos)

        self._csv = open(csv_path, "w", newline="")
        import csv
        self._w = csv.writer(self._csv)
        self._w.writerow(["t", "fl_x", "fl_y", "fl_z", "fl_v",
                          "gt_x", "gt_y", "gt_z", "gt_v",
                          "fus_x", "fus_y", "fus_z",
                          "dx", "dy", "dz", "dxy", "d3d"])
        self.get_logger().info(f"Monitoring → {csv_path} (Ctrl+C to stop)")

    def _mk_cb(self, key):
        def cb(m: Odometry):
            p = m.pose.pose.position
            v = m.twist.twist.linear
            self._latest[key] = (
                t_of(m.header), p.x, p.y, p.z,
                (v.x**2 + v.y**2 + v.z**2) ** 0.5)
        return cb

    def sample(self):
        fl = self._latest["fl"]
        gt = self._latest["gt"]
        fus = self._latest["fus"]
        if fl is None or gt is None:
            return None

        fl_t, fx, fy, fz, fv = fl
        _, gx, gy, gz, gv = gt
        row = [f"{fl_t:.3f}",
               f"{fx:.3f}", f"{fy:.3f}", f"{fz:.3f}", f"{fv:.3f}",
               f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}", f"{gv:.3f}"]
        if fus:
            row += [f"{fus[1]:.3f}", f"{fus[2]:.3f}", f"{fus[3]:.3f}"]
        else:
            row += ["", "", ""]
        dx, dy, dz = fx - gx, fy - gy, fz - gz
        dxy = (dx*dx + dy*dy) ** 0.5
        d3d = (dxy*dxy + dz*dz) ** 0.5
        row += [f"{dx:.3f}", f"{dy:.3f}", f"{dz:.3f}", f"{dxy:.3f}", f"{d3d:.3f}"]
        self._w.writerow(row)
        return fl_t, fv, dxy, dz, d3d

    def close(self):
        self._csv.close()
        self.get_logger().info("CSV closed.")


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/bias_monitor.csv"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rclpy.init()
    n = MonitorBias(csv_path)

    print(f"{'t':>7} {'v':>5} | {'dxy':>6} {'dz':>6} {'d3d':>6}  (FAST-LIO vs 真值)")
    t0 = time.time()
    next_print = 0.0
    try:
        while duration is None or time.time() - t0 < duration:
            rclpy.spin_once(n, timeout_sec=0.02)
            r = n.sample()
            if r and time.time() >= next_print:
                _, v, dxy, dz, d3d = r
                print(f"{r[0]:7.1f} {v:5.2f} | {dxy:6.2f} {dz:+6.2f} {d3d:6.2f}")
                next_print = time.time() + 1.0
    except KeyboardInterrupt:
        pass
    n.close()
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass  # Ctrl+C already tore down the context — ignore
    print(f"\nSaved: {csv_path}")


if __name__ == "__main__":
    main()
