#!/usr/bin/env python3
"""gt_path.py — Publish the Gazebo ground-truth trajectory in the visualization
world frame (anchored at the drone launch position).

The Gazebo truth odom (/gz/ground_truth/odom) is expressed in the Gazebo world
origin (0,0,0), while the visualization world frame is anchored at the drone
launch position (= PX4 ENU / camera_init origin). This node subtracts the spawn
offset so the truth path lines up with FAST-LIO /cloud_registered and SUPER
/cloud_registered_px4 in RViz.

Subscribes:  /gz/ground_truth/odom  (nav_msgs/Odometry, Gazebo truth, gz origin)
Publishes:   /gt_path              (nav_msgs/Path, frame world = launch origin)

Usage:
  python3 gt_path.py --ros-args -p spawn_offset_x:=-4.0 -p spawn_offset_y:=-2.0
      -p spawn_offset_z:=1.15
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MAX_POINTS = 5000


class GtPathNode(Node):
    def __init__(self):
        super().__init__("gt_path")
        self.declare_parameter("input_topic", "/gz/ground_truth/odom")
        self.declare_parameter("output_topic", "/gt_path")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("spawn_offset_x", 0.0)
        self.declare_parameter("spawn_offset_y", 0.0)
        self.declare_parameter("spawn_offset_z", 0.0)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._frame = self.get_parameter("world_frame").value
        self._ox = float(self.get_parameter("spawn_offset_x").value)
        self._oy = float(self.get_parameter("spawn_offset_y").value)
        self._oz = float(self.get_parameter("spawn_offset_z").value)

        self._path = Path()
        self._path.header.frame_id = self._frame

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(Odometry, in_topic, self._cb, qos_be)
        self._pub = self.create_publisher(Path, out_topic, qos_be)
        self.get_logger().info(
            f"gt_path: {in_topic} -> {out_topic} (frame={self._frame}, "
            f"spawn offset=({self._ox},{self._oy},{self._oz}))")

    def _cb(self, m: Odometry):
        self._path.header.stamp = m.header.stamp
        pose = PoseStamped()
        pose.header.frame_id = self._frame
        pose.header.stamp = m.header.stamp
        # Gazebo truth is at the gz origin; shift to the launch-origin world.
        pose.pose.position.x = m.pose.pose.position.x - self._ox
        pose.pose.position.y = m.pose.pose.position.y - self._oy
        pose.pose.position.z = m.pose.pose.position.z - self._oz
        pose.pose.orientation = m.pose.pose.orientation
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
