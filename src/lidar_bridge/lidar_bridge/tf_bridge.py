#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish the TF tree needed to visualize the X500 LiDAR in RViz.

The ros_gz_bridge parameter_bridge only forwards messages - it does NOT
publish TF. Without a transform from RViz's fixed frame to the cloud frame
(lidar_link) nothing is displayed. This node provides that tree:

    world -> base_link    (dynamic, from the bridged /odom)
    base_link -> lidar_link (static; lidar is 0.16 m above base_link)

Usage:
    python3 tf_bridge.py                 # defaults shown below
    python3 tf_bridge.py --ros-args -p odom_topic:=/odom

Requires ROS 2 (Humble) to be sourced.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TfBridge(Node):
    def __init__(self):
        super().__init__('gz_tf_bridge')

        self.declare_parameter('odom_topic', '/lidar_slam/odom')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('lidar_offset_x', 0.0)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 0.16)

        odom_topic = self.get_parameter('odom_topic').value
        world_frame = self.get_parameter('world_frame').value
        base_frame = self.get_parameter('base_frame').value
        lidar_frame = self.get_parameter('lidar_frame').value

        self._world_frame = world_frame
        self._base_frame = base_frame
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # Static base_link -> lidar_link
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id = base_frame
        static.child_frame_id = lidar_frame
        static.transform.translation.x = self.get_parameter('lidar_offset_x').value
        static.transform.translation.y = self.get_parameter('lidar_offset_y').value
        static.transform.translation.z = self.get_parameter('lidar_offset_z').value
        static.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(static)

        # Dynamic world -> base_link from odometry. /lidar_slam/odom is
        # published best_effort/volatile by super_bridge; subscribing RELIABLE
        # would make discovery log an incompatible-QoS warning and the TF
        # would never update.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos)

        self.get_logger().info(
            f'Publishing TF: {world_frame} -> {base_frame} (from {odom_topic}), '
            f'static {base_frame} -> {lidar_frame} (z='
            f'{self.get_parameter("lidar_offset_z").value} m)')

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._world_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
