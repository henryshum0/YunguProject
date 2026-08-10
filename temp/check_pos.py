#!/usr/bin/env python3
"""Compare FAST-LIO /Odometry vs Gazebo ground truth /odom in real time.

Run while the stack is up (drone hovering or flying):

  python3 temp/check_pos.py [seconds]

Expected at hover:
  - z_diff ≈ +0.16 (FAST-LIO reports LiDAR height, 0.16m above base_link)
  - diff  ≈ 0.2-0.4 m (dominated by the fixed height offset)
  - x/y diffs small and stable

If diff grows while flying → FAST-LIO is drifting.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

    rclpy.init()
    n = Node("pos_compare")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

    d = {"fl": None, "gt": None}

    def cb_fl(m):
        d["fl"] = m.pose.pose.position

    def cb_gt(m):
        d["gt"] = m.pose.pose.position

    n.create_subscription(Odometry, "/Odometry", cb_fl, qos)  # FAST-LIO
    n.create_subscription(Odometry, "/odom", cb_gt, qos)      # Gazebo truth

    print(" FAST-LIO (x,y,z)      | Gazebo truth (x,y,z)   | diff(m) | z_diff")
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if d["fl"] and d["gt"]:
            fl, gt = d["fl"], d["gt"]
            diff = ((fl.x - gt.x) ** 2 + (fl.y - gt.y) ** 2 + (fl.z - gt.z) ** 2) ** 0.5
            print(f" ({fl.x:7.2f},{fl.y:7.2f},{fl.z:7.2f}) | "
                  f"({gt.x:7.2f},{gt.y:7.2f},{gt.z:7.2f}) | "
                  f"{diff:6.2f} | {fl.z - gt.z:+5.2f}")
            d["fl"] = d["gt"] = None
        time.sleep(0.3)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
