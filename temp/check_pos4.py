#!/usr/bin/env python3
"""check_pos4.py — Split FAST-LIO bias sources by scenario.

Purpose: determine WHY FAST-LIO deviates from Gazebo ground truth.
The script logs (to stdout + CSV) position, velocity and timestamps for
FAST-LIO / PX4-fusion / ground-truth simultaneously, so we can test:

  Scenario A (ground static):   v ≈ 0. If bias ≈ 0      → timestamps OK, bias
                                                         comes from motion.
                               If bias >> 0 even static → extrinsic / frame
                                                         origin / init problem.
  Scenario C (flying):          bias growing with speed   → timestamp skew /
                                                             latency problem.
                                bias growing with dist    → odom drift.

CSV columns: t, v (truth speed), fl_x fl_y fl_z, fus_x fus_y fus_z,
             gt_x gt_y gt_z, d3d_fl_gt, dxy_fl_gt, dz_fl_gt,
             d3d_fl_fus, ts_imu_lidar (s)

Usage:
  python3 temp/check_pos4.py [seconds] [csv_file]
"""
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

FRAME_ID_DEPTH = 5


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    csv_file = sys.argv[2] if len(sys.argv) > 2 else "/tmp/check_pos4.csv"

    rclpy.init()
    n = Node("pos_compare4")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

    d = {"fl": None, "fus": None, "gt": None, "imu_t": [], "lidar_t": []}

    def cb_fl(m):    d["fl"] = (m.pose.pose.position, m.twist.twist.linear)
    def cb_fus(m):   d["fus"] = (m.pose.pose.position, None)
    def cb_gt(m):    d["gt"] = (m.pose.pose.position, m.twist.twist.linear)
    def cb_imu(m):
        if len(d["imu_t"]) < 3:
            d["imu_t"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    def cb_lidar(m):
        if len(d["lidar_t"]) < 3:
            d["lidar_t"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    n.create_subscription(Odometry, "/Odometry", cb_fl, qos)
    n.create_subscription(Odometry, "/lidar_slam/odom", cb_fus, qos)
    n.create_subscription(Odometry, "/odom", cb_gt, qos)
    n.create_subscription(Imu, "/livox/imu", cb_imu, qos)
    n.create_subscription(PointCloud2, "/x500_lidar/scan/points_timed", cb_lidar, qos)

    # Show timestamps of the first frames
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if d["imu_t"] and d["lidar_t"]:
        print(f"[timestamps] first IMU={d['imu_t'][0]:.4f}  first LiDAR={d['lidar_t'][0]:.4f}"
              f"  diff(IMU-LiDAR)={d['imu_t'][0]-d['lidar_t'][0]:+.4f}s")
    else:
        print("[timestamps] not enough data — check topics!")

    f = open(csv_file, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "v", "fl_x", "fl_y", "fl_z", "fus_x", "fus_y", "fus_z",
                "gt_x", "gt_y", "gt_z", "d3d_fl_gt", "dxy_fl_gt", "dz_fl_gt",
                "d3d_fl_fus"])

    print(f"\n{'t':>6} {'v(m/s)':>7} | {'FL z':>7} {'GT z':>7} | "
          f"{'d3d':>6} {'dxy':>6} {'dz':>6} | {'FL-融合':>6} | 记录中({duration:.0f}s)")
    t0 = time.time()
    last_v = 0.0
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if d["fl"] and d["gt"]:
            fl, fltw = d["fl"]
            gt, gttw = d["gt"]
            fus = d["fus"][0] if d["fus"] else None
            v = gttw.x ** 2 + gttw.y ** 2 + gttw.z ** 2
            v = v ** 0.5 if v > 0 else last_v
            if v > 0:
                last_v = v

            d3d = ((fl.x-gt.x)**2 + (fl.y-gt.y)**2 + (fl.z-gt.z)**2) ** 0.5
            dxy = ((fl.x-gt.x)**2 + (fl.y-gt.y)**2) ** 0.5
            dz = fl.z - gt.z
            dfus = ((fl.x-fus.x)**2 + (fl.y-fus.y)**2 + (fl.z-fus.z)**2) ** 0.5 if fus else float('nan')

            now = time.time() - t0
            print(f"{now:6.1f} {v:7.2f} | {fl.z:7.2f} {gt.z:7.2f} | "
                  f"{d3d:6.2f} {dxy:6.2f} {dz:+6.2f} | {dfus:6.2f}")
            w.writerow([f"{now:.2f}", f"{v:.3f}",
                        f"{fl.x:.3f}", f"{fl.y:.3f}", f"{fl.z:.3f}",
                        *(f"{fus.x:.3f}" if fus else "", f"{fus.y:.3f}" if fus else "",
                          f"{fus.z:.3f}" if fus else ""),
                        f"{gt.x:.3f}", f"{gt.y:.3f}", f"{gt.z:.3f}",
                        f"{d3d:.3f}", f"{dxy:.3f}", f"{dz:.3f}", f"{dfus:.3f}"])
            d["fl"] = d["fus"] = d["gt"] = None
        time.sleep(0.1)

    f.close()
    print(f"\nCSV saved: {csv_file}")

    # Summary: correlation of bias with speed (first vs second half of motion)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
