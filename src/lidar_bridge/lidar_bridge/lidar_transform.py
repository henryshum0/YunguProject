#!/usr/bin/python3
# -*- coding: utf-8 -*-
# NOTE: shebang is /usr/bin/python3 (not `env python3`) on purpose: this system
# has a conda Python 3.14 on PATH that cannot load the cp310 rclpy C extension.
"""lidar_transform.py — Transform one LiDAR cloud into base_link frame.

Single-lidar counterpart of lidar_merge: applies the mounting extrinsic
(lidar frame -> base_link: roll about x + translation) and republishes the
cloud on its own topic. Two instances feed FAST_LIO_MULTI_ROS2's two-lidar
fusion, which requires both input clouds to already be in the IMU frame.

Extrinsics match model.sdf (swan_gamma_v2):
  left  lidar: pose (0, +0.40, 0.05, roll -0.6) relative to base_link
  right lidar: pose (0, -0.40, 0.05, roll +0.6) relative to base_link

Usage (started by start_fastlio.sh):
  ros2 run lidar_bridge lidar_transform \
      -p input_topic:=/swan_gamma_v2/scan_left/points \
      -p output_topic:=/swan_gamma_v2/scan_left/points_base \
      -p t_y:=0.40 -p t_z:=0.05 -p roll:=-0.6
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class LidarTransformNode(Node):
    def __init__(self):
        super().__init__("lidar_transform")
        self.declare_parameter("input_topic", "/swan_gamma_v2/scan_left/points")
        self.declare_parameter("output_topic", "/swan_gamma_v2/scan_left/points_base")
        self.declare_parameter("t_x", 0.0)
        self.declare_parameter("t_y", 0.0)
        self.declare_parameter("t_z", 0.0)
        self.declare_parameter("roll", 0.0)   # rotation about x, rad
        self.declare_parameter("frame_id", "base_link")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        tx = float(self.get_parameter("t_x").value)
        ty = float(self.get_parameter("t_y").value)
        tz = float(self.get_parameter("t_z").value)
        roll = float(self.get_parameter("roll").value)
        self._frame_id = self.get_parameter("frame_id").value

        c, s = math.cos(roll), math.sin(roll)
        self._R = np.array([[1.0, 0.0, 0.0],
                            [0.0, c, -s],
                            [0.0, s, c]], dtype=np.float32)
        self._t = np.array([tx, ty, tz], dtype=np.float32)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, in_topic, self._cb, qos)
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.get_logger().info(
            f"lidar transform: {in_topic} → {out_topic} "
            f"(Rz? no — Rx({roll:.3f}) + t=({tx}, {ty}, {tz}), frame {self._frame_id})")

    def _cb(self, msg):
        n = msg.width * msg.height
        step = msg.point_step
        if n == 0 or step == 0:
            return

        offs = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                offs[f.name] = f.offset
        if len(offs) != 3:
            return

        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, name in enumerate(("x", "y", "z")):
            xyz[:, i] = data[:, offs[name]:offs[name] + 4].copy().view(np.float32).ravel()

        xyz_t = xyz @ self._R.T + self._t

        finite = np.isfinite(xyz_t).all(axis=1)
        if not finite.all():
            xyz_t = xyz_t[finite]
            data = data[finite]
        kept = len(xyz_t)
        if kept == 0:
            return

        out_bytes = bytearray(data.tobytes())
        for i, name in enumerate(("x", "y", "z")):
            col = xyz_t[:, i].tobytes()
            for k in range(kept):
                o = k * step + offs[name]
                out_bytes[o:o + 4] = col[k * 4:(k + 1) * 4]

        out = PointCloud2()
        out.header.frame_id = self._frame_id
        out.header.stamp = msg.header.stamp
        out.height = 1
        out.width = kept
        out.fields = list(msg.fields)
        out.is_bigendian = False
        out.point_step = step
        out.row_step = step * kept
        out.data = bytes(out_bytes)
        out.is_dense = True
        self._pub.publish(out)


def main():
    rclpy.init()
    node = LidarTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
