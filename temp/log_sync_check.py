#!/usr/bin/env python3
"""
log_sync_check.py — Resident node: logs FAST-LIO odom + Gazebo truth + planner cmd.
One process, persistent subscriptions — no repeated DDS discovery.

Usage:
  python3 temp/log_sync_check.py 30 /tmp/coord_sync.csv
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from mars_quadrotor_msgs.msg import PositionCommand


class SyncLogger(Node):
    def __init__(self, duration, out_path):
        super().__init__("sync_logger")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._fl = None
        self._gt = None
        self._cmd = None

        self.create_subscription(Odometry, "/Odometry", self._cb_fl, qos)
        self.create_subscription(Odometry, "/odom", self._cb_gt, qos)
        self.create_subscription(PositionCommand, "/planning/pos_cmd",
                                 self._cb_cmd, qos)

        self._duration = duration
        self._out = open(out_path, "w")
        self._out.write("t,fastlio_x,fastlio_y,fastlio_z,"
                        "truth_x,truth_y,truth_z,cmd_x,cmd_y,cmd_z\n")
        self.get_logger().info(f"Logging {duration}s to {out_path}")

    def _cb_fl(self, m):
        self._fl = m.pose.pose.position

    def _cb_gt(self, m):
        self._gt = m.pose.pose.position

    def _cb_cmd(self, m):
        self._cmd = m.position

    @staticmethod
    def _fmt(p):
        if p is None:
            return "NA,NA,NA"
        return f"{p.x:.3f},{p.y:.3f},{p.z:.3f}"

    def run(self):
        start = time.time()
        while time.time() - start < self._duration:
            row = (f"{time.time():.3f},"
                   f"{self._fmt(self._fl)},"
                   f"{self._fmt(self._gt)},"
                   f"{self._fmt(self._cmd)}\n")
            self._out.write(row)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.4)
        self._out.close()
        self.get_logger().info(f"Done. Saved to {self._out.name}")
        with open(self._out.name) as f:
            lines = f.readlines()[1:]
        n_ok = sum(1 for l in lines if "NA" not in l)
        self.get_logger().info(f"Rows: {len(lines)}, with data: {n_ok}")
        for l in lines:
            if "NA" not in l:
                print(l.strip())


def main():
    rclpy.init()
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/coord_sync.csv"
    node = SyncLogger(duration, out)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
