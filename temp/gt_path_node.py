#!/usr/bin/env python3
"""gt_path_node.py — Publish the Gazebo ground-truth trajectory as a Path.

Subscribes:  /odom  (nav_msgs/Odometry, Gazebo truth, frame world)
Publishes:   /gt_path (nav_msgs/Path, frame world)

Accumulates truth positions into a path for RViz so the ground-truth
trajectory can be compared against FAST-LIO's /path during flight.

Usage:
  python3 temp/gt_path_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MAX_POINTS = 5000  # path memory (5000 pts ≈ 8 min @10Hz truth odom)


class GtPathNode(Node):
    def __init__(self):
        super().__init__("gt_path_node")

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._path = Path()
        self._path.header.frame_id = "world"

        self._sub = self.create_subscription(Odometry, "/odom", self._cb, qos_be)
        self._pub = self.create_publisher(Path, "/gt_path", qos_be)

        self.get_logger().info("Ground-truth path: /odom → /gt_path")

    def _cb(self, m: Odometry):
        self._path.header.stamp = m.header.stamp
        pose = PoseStamped()
        pose.header = m.header
        pose.pose = m.pose.pose
        self._path.poses.append(pose)
        if len(self._path.poses) > MAX_POINTS:
            del self._path.poses[:-MAX_POINTS]
        self._pub.publish(self._path)


def main():
    rclpy.init()
    rclpy.spin(GtPathNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
