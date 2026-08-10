#!/usr/bin/env python3
"""Compare FAST-LIO vs PX4 fusion vs Gazebo truth positions (three-way).

Run while the stack is up (drone hovering or flying):

  python3 temp/check_pos3.py [seconds]

Topics:
  /Odometry        — FAST-LIO (camera_init, ENU)
  /lidar_slam/odom — PX4 EKF2 fusion via super_bridge (ENU)
  /odom            — Gazebo ground truth (world, ENU)
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
    n = Node("pos_compare3")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

    d = {"fl": None, "fus": None, "gt": None}

    def cb_fl(m):    d["fl"] = m.pose.pose.position
    def cb_fus(m):   d["fus"] = m.pose.pose.position
    def cb_gt(m):    d["gt"] = m.pose.pose.position

    n.create_subscription(Odometry, "/Odometry", cb_fl, qos)
    n.create_subscription(Odometry, "/lidar_slam/odom", cb_fus, qos)
    n.create_subscription(Odometry, "/odom", cb_gt, qos)

    print(" FAST-LIO (x,y,z)      | PX4融合 (x,y,z)      | 真值 (x,y,z)       | FL-融合 | FL-真值")
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if d["fl"] and d["fus"] and d["gt"]:
            fl, fus, gt = d["fl"], d["fus"], d["gt"]
            d1 = ((fl.x-fus.x)**2 + (fl.y-fus.y)**2 + (fl.z-fus.z)**2) ** 0.5
            d2 = ((fl.x-gt.x)**2 + (fl.y-gt.y)**2 + (fl.z-gt.z)**2) ** 0.5
            print(f" ({fl.x:7.2f},{fl.y:7.2f},{fl.z:7.2f}) | "
                  f"({fus.x:7.2f},{fus.y:7.2f},{fus.z:7.2f}) | "
                  f"({gt.x:7.2f},{gt.y:7.2f},{gt.z:7.2f}) | "
                  f"{d1:5.2f}  | {d2:5.2f}")
            d["fl"] = d["fus"] = d["gt"] = None
        time.sleep(0.3)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
