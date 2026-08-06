#!/usr/bin/env python3
"""
cloud_to_world.py — Transform LiDAR point cloud from lidar_link frame to world frame.

ROG-Map (SUPER) assumes the input point cloud is ALREADY in world coordinates
(it does not apply any pose transform internally). But Gazebo publishes the
cloud in the lidar_link frame (attached to the drone). This node transforms
every point to world using the latest odometry pose.

  p_world = R_wb * (R_bl * p_lidar + t_bl) + t_wb

  R_bl / t_bl : lidar_link → base_link (model.sdf: 0 0 0.16, no rotation)
  R_wb / t_wb : base_link → world (from odom)

Usage:
  python3 temp/cloud_to_world.py --ros-args \
    -p cloud_topic:=/x500_lidar/scan/points \
    -p odom_topic:=/odom \
    -p out_topic:=/x500_lidar/scan/points_world
"""

import struct
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry


def quat_to_rotmat(q):
    """(x,y,z,w) → 3x3 rotation matrix."""
    x, y, z, w = q
    return [
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ]


def mat_vec(R, v):
    return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
            R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
            R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]


class CloudToWorld(Node):
    def __init__(self):
        super().__init__("cloud_to_world")

        self.declare_parameter("cloud_topic", "/x500_lidar/scan/points")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("out_topic", "/x500_lidar/scan/points_world")
        self.declare_parameter("lidar_offset_z", 0.16)   # lidar_link → base_link

        cloud_topic = self.get_parameter("cloud_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        out_topic = self.get_parameter("out_topic").value
        self._z_off = self.get_parameter("lidar_offset_z").value

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom = None
        self._cloud_sub = self.create_subscription(PointCloud2, cloud_topic,
                                                   self._cloud_cb, qos_be)
        self._odom_sub = self.create_subscription(Odometry, odom_topic,
                                                  self._odom_cb, qos_be)
        self._pub = self.create_publisher(PointCloud2, out_topic, qos_be)
        self.get_logger().info(
            f"cloud_to_world: {cloud_topic} + {odom_topic} → {out_topic}")

    def _odom_cb(self, m):
        self._odom = m

    def _cloud_cb(self, cloud):
        if self._odom is None:
            return

        # Pose from odom (world ← base_link)
        t_wb = [self._odom.pose.pose.position.x,
                self._odom.pose.pose.position.y,
                self._odom.pose.pose.position.z]
        q = (self._odom.pose.pose.orientation.x,
             self._odom.pose.pose.orientation.y,
             self._odom.pose.pose.orientation.z,
             self._odom.pose.pose.orientation.w)
        R_wb = quat_to_rotmat(q)

        # Point layout
        fields = {f.name: (f.offset, f.datatype, f.count) for f in cloud.fields}
        n_pts = cloud.width * cloud.height
        ps = cloud.point_step
        data = cloud.data
        nbytes = max(f[0] + 4 for f in fields.values()) if fields else ps

        def get_xyz(i):
            o = i * ps
            x = struct.unpack_from('<f', data, o + fields['x'][0])[0]
            y = struct.unpack_from('<f', data, o + fields['y'][0])[0]
            z = struct.unpack_from('<f', data, o + fields['z'][0])[0]
            return x, y, z

        # Transform each point: p_world = R_wb*(p_lidar + [0,0,z_off]) + t_wb
        new_data = bytearray(data)  # copy in place
        for i in range(n_pts):
            x, y, z = get_xyz(i)
            pl = [x, y, z + self._z_off]
            pw = mat_vec(R_wb, pl)
            px, py, pz = pw[0] + t_wb[0], pw[1] + t_wb[1], pw[2] + t_wb[2]
            o = i * ps
            struct.pack_into('<f', new_data, o + fields['x'][0], px)
            struct.pack_into('<f', new_data, o + fields['y'][0], py)
            struct.pack_into('<f', new_data, o + fields['z'][0], pz)

        out = PointCloud2()
        out.header = cloud.header
        out.header.frame_id = "world"
        out.height = cloud.height
        out.width = cloud.width
        out.fields = cloud.fields
        out.is_bigendian = cloud.is_bigendian
        out.point_step = cloud.point_step
        out.row_step = cloud.row_step
        out.is_dense = cloud.is_dense
        out.data = bytes(new_data)
        self._pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CloudToWorld())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
