#!/usr/bin/env python3
"""world_map_node.py — Accumulate the world-frame registered cloud into a map.

Subscribes:  /cloud_registered  (PointCloud2, world frame, one scan per msg)
Publishes:   /world_map        (PointCloud2, world frame, accumulated map)

super_bridge republishes the merged lidar scan transformed into the world
frame (/cloud_registered) — one *current scan* per message. This node voxel-
downsamples each scan and merges it into a growing map, so RViz can show a
persistent registered point cloud map instead of just the live scan.

The map is a *local* map that follows the drone, so the initial point cloud
never piles up and old regions are forgotten as the drone flies away:
  - dwell gate: while the drone has moved less than start_move_threshold m
    from where this node started (still on the ground / arming at spawn),
    scans are not accumulated at all — no dense blob at the takeoff point;
  - range forgetting: chunks farther than forget_range m from the drone are
    dropped, so the start region disappears once the drone flies away;
  - new points are only kept for voxels not seen before (voxel_size);
  - the max_points sliding window stays as a backstop cap.

Usage:
  python3 src/offboard/scripts/world_map_node.py
  # optional params:
  #   -p voxel_size:=0.2 -p publish_period:=3.0 -p max_points:=5000000
  #   -p start_move_threshold:=1.0 -p forget_range:=100.0
  #   -p pcd_save_path:=map.pcd
"""

import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

DEFAULT_VOXEL = 0.15
DEFAULT_MAX_POINTS = 500000
DEFAULT_PERIOD = 2.0
DEFAULT_START_MOVE = 1.0        # m — drone movement before the map starts accumulating
DEFAULT_FORGET_RANGE = 15.0    # m — chunks farther than this from the drone are dropped

# Voxel key packing: 21 bits per axis in one int64 (range ±1M voxels per
# axis — ±157 km at 0.15 m). Keeps membership checks in a plain set, which
# is ~4x leaner than tuples of np.int64 and much faster.
_VOX_SHIFT = 21
_VOX_MASK = (1 << _VOX_SHIFT) - 1


def pack_keys(vox: np.ndarray) -> np.ndarray:
    """Pack an (N,3) int64 voxel-coordinate array into (N,) int64 keys."""
    v = vox.astype(np.int64)
    return ((v[:, 0] & _VOX_MASK) << 42) | ((v[:, 1] & _VOX_MASK) << 21) | (v[:, 2] & _VOX_MASK)


class WorldMapNode(Node):
    def __init__(self):
        super().__init__("world_map_node")

        self.declare_parameter("in_topic", "/cloud_registered")
        self.declare_parameter("out_topic", "/world_map")
        self.declare_parameter("voxel_size", DEFAULT_VOXEL)
        self.declare_parameter("publish_period", DEFAULT_PERIOD)
        self.declare_parameter("max_points", DEFAULT_MAX_POINTS)
        self.declare_parameter("pcd_save_path", "")
        self.declare_parameter("odom_topic", "/lidar_slam/odom")
        self.declare_parameter("start_move_threshold", DEFAULT_START_MOVE)
        self.declare_parameter("forget_range", DEFAULT_FORGET_RANGE)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self._voxel = float(self.get_parameter("voxel_size").value)
        self._period = float(self.get_parameter("publish_period").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._pcd_path = str(self.get_parameter("pcd_save_path").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._start_move = float(self.get_parameter("start_move_threshold").value)
        self._forget_range = float(self.get_parameter("forget_range").value)

        qos_be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._cb, qos_be)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, qos_be)
        self._pub = self.create_publisher(PointCloud2, out_topic, qos_be)

        # Map storage: chunks of (N,3) float32 arrays + packed-key set.
        # Chunks are appended in arrival order; on cap the OLDEST chunk(s)
        # are dropped (sliding window) so the map never stops accumulating.
        self._chunks: list[np.ndarray] = []
        self._chunk_keys: list[np.ndarray] = []
        self._keys: set = set()
        self._total = 0
        self._dropped = 0

        # Drone world position (from /lidar_slam/odom) — used by the dwell
        # gate and the range forgetting. The gate latches open once the drone
        # first moves start_move_threshold m, so returning near the spawn
        # point mid-flight does not pause mapping again.
        self._odom_pos: np.ndarray | None = None
        self._start_pos: np.ndarray | None = None
        self._gate_open = False
        self._forget_dropped = 0

        # Stats for the periodic growth log.
        self._stats_total = 0
        self._stats_last = time.monotonic()

        self._timer = self.create_timer(self._period, self._publish)
        self.get_logger().info(
            f"world map: {in_topic} → {out_topic} "
            f"(voxel {self._voxel} m, cap {self._max_points} pts, "
            f"dwell gate {self._start_move} m, forget >{self._forget_range} m)")

    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry):
        self._odom_pos = np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z],
                                  dtype=np.float64)

    # ------------------------------------------------------------------ #
    def _cb(self, msg: PointCloud2):
        if msg.width * msg.height == 0:
            return

        # Dwell gate: while the drone sits on the ground at spawn (arming,
        # waiting for takeoff), every scan covers the same surroundings and
        # would pile up a dense blob at the start that never goes away. Do
        # not accumulate until the drone has actually moved
        # start_move_threshold m from where this node started; the gate
        # latches open for the rest of the run. If no odom is seen at all
        # (bridge down), fall back to always accumulating.
        if self._odom_pos is not None and not self._gate_open:
            if self._start_pos is None:
                self._start_pos = self._odom_pos.copy()
            if np.linalg.norm(self._odom_pos - self._start_pos) < self._start_move:
                return
            self._gate_open = True

        # Locate float32 x/y/z fields (same defensive approach as super_bridge).
        x_off = y_off = z_off = -1
        for f in msg.fields:
            if f.name == "x" and f.datatype == PointField.FLOAT32:
                x_off = f.offset
            elif f.name == "y" and f.datatype == PointField.FLOAT32:
                y_off = f.offset
            elif f.name == "z" and f.datatype == PointField.FLOAT32:
                z_off = f.offset
        if x_off < 0 or y_off < 0 or z_off < 0:
            return

        floats_per_point = msg.point_step // 4
        n = msg.width * msg.height
        pts = np.frombuffer(msg.data, dtype=np.float32,
                            count=n * floats_per_point).reshape(n, floats_per_point)
        xyz = pts[:, [x_off // 4, y_off // 4, z_off // 4]]

        # Local voxel downsample: keep the first point of each occupied voxel.
        keys = pack_keys(np.floor(xyz / self._voxel))
        _, first = np.unique(keys, return_index=True)
        keys, new_pts = keys[first], xyz[first]

        # Keep only voxels not yet in the map.
        keep = [i for i, k in enumerate(keys) if k not in self._keys]
        if not keep:
            return
        keys = keys[keep]
        new_pts = new_pts[keep]

        # Sliding window: drop the OLDEST chunks until the new frame fits,
        # so the map keeps accumulating instead of freezing at the cap.
        while self._total + len(new_pts) > self._max_points and self._chunks:
            dropped_keys = self._chunk_keys.pop(0)
            dropped_pts = self._chunks.pop(0)
            self._keys.difference_update(dropped_keys.tolist())
            self._total -= len(dropped_pts)
            self._dropped += len(dropped_pts)
        if self._total + len(new_pts) > self._max_points:
            # Single frame larger than the cap (never in practice) — truncate.
            new_pts = new_pts[:self._max_points - self._total]
            keys = keys[:len(new_pts)]
            if len(new_pts) == 0:
                return

        chunk = np.ascontiguousarray(new_pts, dtype=np.float32)
        self._chunks.append(chunk)
        self._chunk_keys.append(keys)
        self._keys.update(keys.tolist())
        self._total += len(new_pts)

    # ------------------------------------------------------------------ #
    def _prune_far_chunks(self):
        """Drop chunks farther than forget_range from the drone.

        Makes /world_map a local map that follows the drone: the initial
        point cloud (takeoff area, start region) and any other region behind
        the flight are forgotten once the drone flies beyond forget_range,
        instead of lingering forever.
        """
        if self._odom_pos is None or not self._chunks:
            return
        drop = {i for i, c in enumerate(self._chunks)
                if np.linalg.norm(c.mean(axis=0) - self._odom_pos) > self._forget_range}
        if not drop:
            return
        removed = sum(len(self._chunks[i]) for i in drop)
        self._chunks = [c for i, c in enumerate(self._chunks) if i not in drop]
        self._chunk_keys = [k for i, k in enumerate(self._chunk_keys) if i not in drop]
        if self._chunk_keys:
            self._keys = set().union(*(k.tolist() for k in self._chunk_keys))
        else:
            self._keys = set()
        self._total -= removed
        self._forget_dropped += removed
        self.get_logger().info(
            f"map: forgot {removed} pts ({len(drop)} chunks) beyond "
            f"{self._forget_range} m from drone ({self._total} pts left)")

    # ------------------------------------------------------------------ #
    def _build_msg(self) -> PointCloud2:
        all_pts = np.concatenate(self._chunks) if self._chunks else np.empty((0, 3), np.float32)
        msg = PointCloud2()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(all_pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(all_pts)
        msg.data = all_pts.tobytes()
        msg.is_dense = True
        return msg

    def _publish(self):
        if not self._chunks:
            return
        # Forget regions the drone has flown away from (runs before the cap
        # logic: range forgetting is the primary mechanism, the max_points
        # sliding window is only a backstop).
        self._prune_far_chunks()
        if not self._chunks:
            return
        # Periodic growth stats (every ~30 s) so it's visible the map is
        # still accumulating during flight.
        now = time.monotonic()
        if now - self._stats_last >= 30.0:
            self._stats_last = now
            delta = self._total - self._stats_total
            self._stats_total = self._total
            self.get_logger().info(
                f"map: {self._total} pts (+{delta} since last report, "
                f"{self._dropped} dropped to stay under cap, "
                f"{self._forget_dropped} forgotten beyond range)")
        self._pub.publish(self._build_msg())

    def save_pcd(self):
        """Write the accumulated map as a binary PCD (PCL-readable)."""
        if not self._chunks:
            return
        path = Path(self._pcd_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        all_pts = np.concatenate(self._chunks)
        n = len(all_pts)
        with open(path, "wb") as f:
            f.write(
                f"# .PCD v0.7 - Point Cloud Data file format\n"
                f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                f"COUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n"
                f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
                .encode())
            f.write(all_pts.tobytes())
        # Called after the ROS context may already be torn down — plain print.
        print(f"[world_map_node] map saved: {path} ({n} pts)")


def main():
    rclpy.init()
    node = WorldMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Ctrl+C in Humble raises ExternalShutdownException after the
        # context is torn down; save the map first if requested.
        if node._pcd_path:
            node.save_pcd()
    finally:
        # rclpy already shut the context down on SIGINT — guard both calls.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
