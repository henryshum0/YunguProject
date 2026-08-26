#!/usr/bin/env python3
"""fastlio_visual.py — Publish FAST-LIO's registered cloud in the visualization
world frame for RViz.

FAST-LIO publishes /cloud_registered in its camera_init frame, whose origin is
the drone launch position — exactly the visualization world frame chosen for
RViz. So this node simply relabels the frame to `world` (geometry unchanged),
and optionally forwards FAST-LIO /Odometry as a world-frame odom for the drone
path display.

Subscribes:  /cloud_registered  (sensor_msgs/PointCloud2, frame camera_init)
             /Odometry          (nav_msgs/Odometry, camera_init -> body)
Publishes:   /fastlio_cloud     (sensor_msgs/PointCloud2, frame world)
             /fastlio_odom      (nav_msgs/Odometry, world -> body)
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class FastlioVisual(Node):
    def __init__(self):
        super().__init__("fastlio_visual")
        self.declare_parameter("cloud_in_topic", "/cloud_registered")
        self.declare_parameter("cloud_out_topic", "/fastlio_cloud")
        self.declare_parameter("odom_in_topic", "/Odometry")
        self.declare_parameter("odom_out_topic", "/fastlio_odom")
        self.declare_parameter("world_frame", "world")

        c_in = self.get_parameter("cloud_in_topic").value
        c_out = self.get_parameter("cloud_out_topic").value
        o_in = self.get_parameter("odom_in_topic").value
        o_out = self.get_parameter("odom_out_topic").value
        self._frame = self.get_parameter("world_frame").value

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cloud_sub = self.create_subscription(
            PointCloud2, c_in, self._cloud_cb, qos_be)
        self._odom_sub = self.create_subscription(Odometry, o_in, self._odom_cb, qos_be)
        self._cloud_pub = self.create_publisher(PointCloud2, c_out, qos_be)
        self._odom_pub = self.create_publisher(Odometry, o_out, qos_be)
        self.get_logger().info(
            f"fastlio_visual: {c_in} -> {c_out}, {o_in} -> {o_out} "
            f"(frame={self._frame})")

    def _cloud_cb(self, msg: PointCloud2):
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self._frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = bytes(msg.data)
        self._cloud_pub.publish(out)

    def _odom_cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self._frame
        out.child_frame_id = "body"
        out.pose = msg.pose
        out.twist = msg.twist
        self._odom_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(FastlioVisual())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
