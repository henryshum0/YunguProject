#!/usr/bin/python3
# -*- coding: utf-8 -*-
# NOTE: shebang is /usr/bin/python3 (not `env python3`) on purpose: this system
# has a conda Python 3.14 on PATH that cannot load the cp310 rclpy C extension.
"""lidar_merge.py — Fuse two side-mounted LiDAR clouds into one scan.

Mirrors the real hardware: one LiDAR on each side of the drone (tilted down
~35°). Each cloud is transformed into a common frame (base_link) using its
mounting extrinsics, then concatenated into a single PointCloud2 for FAST-LIO.

The extrinsics below match model.sdf (swan_gamma_v2):
  left  lidar: pose (0, +0.40, 0.05, roll -0.6) relative to base_link
  right lidar: pose (0, -0.40, 0.05, roll +0.6) relative to base_link

Publishes a fused frame whenever either side delivers a new scan (the other
side's latest is reused), so the output rate stays at the scan rate.

Usage (started by start_fastlio.sh):
  python3 utils/fastlio/lidar_merge.py
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

# Mounting extrinsics (lidar frame -> base_link), matching model.sdf.
# roll: rotation about the forward (x) axis, rad.
EXTRINSICS = {
    "left":  {"t": (0.0, 0.40, 0.05), "roll": -0.6},
    "right": {"t": (0.0, -0.40, 0.05), "roll": +0.6},
}

TOPIC_LEFT = "/swan_gamma_v2/scan_left/points_timed"
TOPIC_RIGHT = "/swan_gamma_v2/scan_right/points_timed"
TOPIC_OUT = "/swan_gamma_v2/scan/points_fused"


def _rotation_matrix(roll):
    c, s = math.cos(roll), math.sin(roll)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]], dtype=np.float32)


def _stamp_key(t):
    """builtin_interfaces.msg.Time has no ordering operators — compare tuples."""
    return (t.sec, t.nanosec)


class LidarMergeNode(Node):
    def __init__(self):
        super().__init__("lidar_merge")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._latest = {"left": None, "right": None}
        self.create_subscription(PointCloud2, TOPIC_LEFT,
                                 lambda m: self._cb("left", m), qos)
        self.create_subscription(PointCloud2, TOPIC_RIGHT,
                                 lambda m: self._cb("right", m), qos)
        self._pub = self.create_publisher(PointCloud2, TOPIC_OUT, qos)
        self.get_logger().info(
            f"lidar merge: {TOPIC_LEFT} + {TOPIC_RIGHT} → {TOPIC_OUT} (base_link)")

    # ---------------------------------------------------------------- utils
    @staticmethod
    def _offsets(msg):
        offs = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                offs[f.name] = f.offset
        return offs

    def _transform_xyz(self, msg, side):
        """Return (bytes, kept) — cloud bytes with x/y/z transformed into
        base_link, and the kept point count. gz gpu_lidar emits NaN
        coordinates for no-return rays; those points are dropped so the
        fused cloud stays finite for FAST-LIO."""
        ext = EXTRINSICS[side]
        R = _rotation_matrix(ext["roll"])
        t = np.array(ext["t"], dtype=np.float32)

        n = msg.width * msg.height
        step = msg.point_step
        offs = self._offsets(msg)
        if len(offs) != 3 or n == 0 or step == 0:
            return None, 0

        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, name in enumerate(("x", "y", "z")):
            xyz[:, i] = data[:, offs[name]:offs[name] + 4].copy().view(np.float32).ravel()

        xyz_t = xyz @ R.T + t  # (n,3) rotated + translated

        finite = np.isfinite(xyz_t).all(axis=1)
        if not finite.all():
            xyz_t = xyz_t[finite]
            data = data[finite]
        kept = len(xyz_t)

        out = bytearray(data.tobytes())
        for i, name in enumerate(("x", "y", "z")):
            col = xyz_t[:, i].tobytes()
            for k in range(kept):
                o = k * step + offs[name]
                out[o:o + 4] = col[k * 4:(k + 1) * 4]
        return bytes(out), kept

    # ------------------------------------------------------------------ sub
    def _cb(self, side, msg):
        payload, kept = self._transform_xyz(msg, side)
        if payload is None or kept == 0:
            return
        self._latest[side] = (payload, kept, msg.header.stamp)

        # Merge with the other side's latest (if any) and publish.
        parts = []
        total = 0
        stamp = None
        for s in ("left", "right"):
            item = self._latest[s]
            if item is None:
                continue
            payload_s, kept_s, stamp_s = item
            parts.append(payload_s)
            total += kept_s
            if stamp is None or _stamp_key(stamp_s) > _stamp_key(stamp):
                stamp = stamp_s
        if not parts or total == 0:
            return

        out = PointCloud2()
        out.header.frame_id = "base_link"
        out.header.stamp = stamp
        out.height = 1
        out.width = total
        out.fields = list(msg.fields)
        out.is_bigendian = False
        out.point_step = msg.point_step
        out.row_step = msg.point_step * total
        out.data = b"".join(parts)
        out.is_dense = True
        self._pub.publish(out)


def main():
    rclpy.init()
    node = LidarMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
