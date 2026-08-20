#!/usr/bin/python3
# -*- coding: utf-8 -*-
# NOTE: shebang is /usr/bin/python3 (not `env python3`) on purpose: this system
# has a conda Python 3.14 on PATH that cannot load the cp310 rclpy C extension.
"""monitor.py — Real-time fusion monitor: Gazebo GT vs FAST-LIO vs PX4 fused.

One window, four 2D panels (no 3D — avoids the broken mpl_toolkits on this box):

  ┌─────────────────────────────┬─────────────────────────────┐
  │ XY top-down (follows drone) │ z vs time (drift visible)   │
  ├─────────────────────────────┼─────────────────────────────┤
  │ horizontal error vs GT      │ vertical error vs GT        │
  ├─────────────────────────────┴─────────────────────────────┤
  │ process CPU/mem (htop-like, sampled from /proc)           │
  └───────────────────────────────────────────────────────────┘

Topics (all best_effort, matching the rest of the project):
  /odom            Gazebo ground truth, nav_msgs/Odometry, frame world
  /Odometry        FAST-LIO output,    nav_msgs/Odometry, frame camera_init
  /lidar_slam/odom PX4 EKF2 fused,     nav_msgs/Odometry, frame world

FAST-LIO is transformed from camera_init into world via the static TF
world -> camera_init published by offboard.launch.py when use_fastlio is
enabled (spawn pose + lidar offset), so all three streams are compared in
one frame.

Run it while the stack is up, in any terminal:

  ros2 run flight_monitor monitor

Deps (already installed by ./install_deps.sh + the matplotlib fix):
numpy, matplotlib>=3.9, python3-tk.
"""

import math
import os
import threading
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

MAX_PTS = 6000        # points kept per trajectory
MAX_ERR = 4000        # points kept per error series
FOLLOW_RADIUS = 25.0  # [m] XY view keeps the drone centered in this window
ANIM_MS = 100         # redraw period [ms]
N_PER_SEC = 20        # expected points per second per source (for time windows)

COLORS = {"gt": "green", "px4": "red", "fastlio": "blue"}
LABELS = {"gt": "GT", "px4": "PX4 fused", "fastlio": "FAST-LIO"}

# ------------------------------------------------------------------ process
# htop-like process tracking: name substrings to watch, sample interval.
PROC_NAMES = ("px4", "gz sim", "fastlio_mapping", "fastlio_px4_bridge",
              "add_time_field", "super_bridge",
              "offboard_node", "MicroXRCEAgent", "monitor.py")
CPU_TICKS = 20        # refresh CPU panel every 20 frames (2 s at 100 ms)
CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


def _proc_snapshot():
    """[(name, cpu_pct, rss_mb)] from /proc — cpu% over the last 2 s."""
    now = time.monotonic()
    procs = []  # (pid, name)
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/comm") as f:
                comm = f.read().strip()
        except OSError:
            continue
        if any(n in comm for n in PROC_NAMES):
            procs.append((int(entry), comm))
    # remember previous per-pid cpu time for the delta
    state = getattr(_proc_snapshot, "prev", {})
    rows = []
    for pid, name in procs:
        try:
            with open(f"/proc/{pid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
            proc_t = (int(fields[11]) + int(fields[12])) / CLK_TCK  # utime+stime
            with open(f"/proc/{pid}/statm") as f:
                rss_pages = int(f.read().split()[1])
        except (OSError, ValueError, IndexError):
            continue
        rss_mb = rss_pages * os.sysconf("SC_PAGE_SIZE") / 1e6
        prev = state.get(pid)
        if prev is not None and now > prev[0]:
            cpu = (proc_t - prev[1]) / (now - prev[0]) * 100.0
            rows.append((name, min(cpu, 999.9), rss_mb))
        state[pid] = (now, proc_t)
    _proc_snapshot.prev = {pid: v for pid, v in state.items()
                           if pid in dict(procs)}
    rows.sort(key=lambda r: -r[1])
    return rows, now


class FusionMonitorNode(Node):
    def __init__(self):
        super().__init__("fusion_monitor")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # (x, y, z, yaw, t_ns) per source, trimmed to MAX_PTS
        self.traj = {"gt": deque(), "px4": deque(), "fastlio": deque()}
        # error series vs the latest GT sample at arrival: (t_ns, horiz, vert)
        self.err = {"px4": deque(), "fastlio": deque()}
        self._gt_latest = None
        # subscriptions (rclpy spin thread) mutate the deques while the GUI
        # thread reads them — a lock makes the snapshots safe.
        self.lock = threading.Lock()

        self.create_subscription(Odometry, "/odom", self._cb_gt, qos)
        self.create_subscription(Odometry, "/lidar_slam/odom", self._cb_px4, qos)
        self.create_subscription(Odometry, "/Odometry", self._cb_fastlio, qos)
        self.get_logger().info(
            "fusion monitor: /odom + /lidar_slam/odom + /Odometry (FAST-LIO -> world via TF)")

    # ------------------------------------------------------------------ subs
    def _cb_gt(self, m: Odometry):
        p = m.pose.pose.position
        self._push("gt", p.x, p.y, p.z, _yaw_of(m.pose.pose.orientation))
        self._gt_latest = (p.x, p.y, p.z)

    def _cb_px4(self, m: Odometry):
        p = m.pose.pose.position
        self._push("px4", p.x, p.y, p.z, _yaw_of(m.pose.pose.orientation))
        self._push_err("px4", p.x, p.y, p.z)

    def _cb_fastlio(self, m: Odometry):
        # FAST-LIO lives in camera_init; move it into world via the static TF
        # published by start_fastlio.sh before comparing with the other two.
        try:
            t = self.tf_buffer.lookup_transform("world", "camera_init", Time())
        except Exception:
            return  # TF not up yet — try again on the next message
        p = m.pose.pose.position
        x = p.x + t.transform.translation.x
        y = p.y + t.transform.translation.y
        z = p.z + t.transform.translation.z
        self._push("fastlio", x, y, z, _yaw_of(m.pose.pose.orientation))
        self._push_err("fastlio", x, y, z)

    # ----------------------------------------------------------------- utils
    def _push(self, key, x, y, z, yaw):
        with self.lock:
            dq = self.traj[key]
            dq.append((x, y, z, yaw, self.get_clock().now().nanoseconds))
            while len(dq) > MAX_PTS:
                dq.popleft()

    def _push_err(self, key, x, y, z):
        gt = self._gt_latest
        if gt is None:
            return
        d = (x - gt[0], y - gt[1], z - gt[2])
        with self.lock:
            dq = self.err[key]
            dq.append((self.get_clock().now().nanoseconds,
                       math.hypot(d[0], d[1]), abs(d[2])))
            while len(dq) > MAX_ERR:
                dq.popleft()


def _yaw_of(q) -> float:
    """Quaternion → yaw [rad], ENU (rotation about +z)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _arr(key, node):
    with node.lock:
        data = list(node.traj[key])
    if not data:
        return np.zeros((0, 5))
    return np.array(data)


def main():
    rclpy.init()
    node = FusionMonitorNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fig = plt.figure(figsize=(15, 11))
    fig.canvas.manager.set_window_title("Fusion Monitor — GT / FAST-LIO / PX4 fused")
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 0.9])

    ax_xy = fig.add_subplot(gs[0, 0])
    ax_z = fig.add_subplot(gs[0, 1])
    ax_he = fig.add_subplot(gs[1, 0])
    ax_ve = fig.add_subplot(gs[1, 1])
    ax_cpu = fig.add_subplot(gs[2, :])

    t0 = None  # wall-clock reference for time axes

    def update(_frame):
        nonlocal t0
        if t0 is None:
            t0 = node.get_clock().now().nanoseconds

        # ------------------------------------------------------------ XY
        ax_xy.clear()
        ax_xy.set_title("Top-down XY (world ENU, follows GT)")
        ax_xy.set_xlabel("x [m] (east)"); ax_xy.set_ylabel("y [m] (north)")
        ax_xy.grid(True); ax_xy.set_aspect("equal")

        for key in ("gt", "px4", "fastlio"):
            arr = _arr(key, node)
            if len(arr) == 0:
                continue
            ax_xy.plot(arr[:, 0], arr[:, 1], lw=1.2, alpha=0.85,
                       color=COLORS[key], label=LABELS[key])
            px, py, yaw = arr[-1, 0], arr[-1, 1], arr[-1, 3]
            ax_xy.plot(px, py, "o", color=COLORS[key], ms=7)
            ax_xy.annotate("", xy=(px + 1.6 * math.cos(yaw), py + 1.6 * math.sin(yaw)),
                           xytext=(px, py),
                           arrowprops=dict(arrowstyle="->", color=COLORS[key], lw=1.5))

        ax_xy.legend(loc="lower right", fontsize=8)

        gt = _arr("gt", node)
        if len(gt):
            cx, cy = gt[-1, 0], gt[-1, 1]
            ax_xy.set_xlim(cx - FOLLOW_RADIUS, cx + FOLLOW_RADIUS)
            ax_xy.set_ylim(cy - FOLLOW_RADIUS, cy + FOLLOW_RADIUS)
        else:
            ax_xy.set_xlim(-FOLLOW_RADIUS, FOLLOW_RADIUS)
            ax_xy.set_ylim(-FOLLOW_RADIUS, FOLLOW_RADIUS)

        # live readout (positions, in the XY corner)
        info = []
        for key in ("gt", "px4", "fastlio"):
            arr = _arr(key, node)
            if len(arr):
                info.append(f"{LABELS[key]}: ({arr[-1,0]:6.2f}, {arr[-1,1]:6.2f}, {arr[-1,2]:6.2f})")
        if info:
            ax_xy.text(0.02, 0.98, "\n".join(info), transform=ax_xy.transAxes,
                       va="top", fontsize=8,
                       bbox=dict(boxstyle="round", fc="white", alpha=0.7))

        # -------------------------------------------------------- z vs t
        ax_z.clear()
        ax_z.set_title("Height z vs time (world)")
        ax_z.set_xlabel("time [s]"); ax_z.set_ylabel("z [m]")
        ax_z.grid(True)
        for key in ("gt", "px4", "fastlio"):
            arr = _arr(key, node)
            if len(arr) < 2:
                continue
            t = (arr[:, 4] - t0) * 1e-9
            ax_z.plot(t, arr[:, 2], lw=1.2, color=COLORS[key], label=LABELS[key])
        ax_z.legend(loc="upper right", fontsize=8)

        # -------------------------------------------------- error panels
        for ax, field, title in ((ax_he, 1, "Horizontal error vs GT"),
                                 (ax_ve, 2, "Vertical error (|dz|) vs GT")):
            ax.clear()
            ax.set_title(title)
            ax.set_xlabel("time [s]"); ax.set_ylabel("error [m]")
            ax.grid(True)
            with node.lock:
                err_snap = {k: list(v) for k, v in node.err.items()}
            for key, style in (("px4", "-"), ("fastlio", "--")):
                dq = err_snap[key]
                if len(dq) < 2:
                    continue
                t = [(ts - t0) * 1e-9 for ts, _, _ in dq]
                v = [e[field] for e in dq]
                ax.plot(t, v, style, lw=1.0, color=COLORS[key],
                        label=f"{LABELS[key]}")
            ax.legend(loc="upper left", fontsize=7)

        # --------------------------------------------------- process CPU
        tick = getattr(update, "_tick", 0) + 1
        update._tick = tick
        if tick % CPU_TICKS == 1:
            rows, sample_t = _proc_snapshot()
            update._proc_rows = rows
        else:
            sample_t = time.monotonic()

        ax_cpu.clear()
        ax_cpu.set_title("Process CPU % (top, 2 s window)  —  memory in MB")
        ax_cpu.set_xlabel("CPU % (one bar per core = 100%)")
        ax_cpu.grid(True, axis="x", alpha=0.4)
        rows = getattr(update, "_proc_rows", [])
        if rows:
            names = [f"{r[0]} [{r[2]:.0f} MB]" for r in rows[:8]]
            cpus = [r[1] for r in rows[:8]]
            y = np.arange(len(names))[::-1]
            bars = ax_cpu.barh(y, cpus, color="#d62728", alpha=0.85)
            for yi, b, cpu in zip(y, bars, cpus):
                ax_cpu.text(b.get_width() + 2, yi, f"{cpu:.0f}%",
                            va="center", fontsize=8)
            ax_cpu.set_yticks(y, names, fontsize=8)
            ax_cpu.set_xlim(0, max(max(cpus) * 1.15, 50))
            try:
                ncores = os.cpu_count() or 1
                ax_cpu.axvline(ncores, color="gray", ls="--", lw=0.8)
                ax_cpu.text(ncores, len(names) - 0.5, f"{ncores} cores",
                            fontsize=7, color="gray", ha="left")
            except Exception:
                pass
            try:
                with open("/proc/loadavg") as f:
                    load = f.read().split()[:3]
                ax_cpu.text(0.995, 0.05, f"load: {' '.join(load)}",
                            transform=ax_cpu.transAxes, ha="right",
                            fontsize=8, bbox=dict(boxstyle="round",
                                                  fc="white", alpha=0.7))
            except OSError:
                pass

        fig.tight_layout()
        return ()

    anim = FuncAnimation(fig, update, interval=ANIM_MS, blit=False,
                         cache_frame_data=False)
    # Graceful shutdown: stop rclpy and join the spin thread BEFORE the
    # interpreter exits, or the executor thread's std::thread destructor
    # throws "terminate called without an active exception" (core dump).
    def _on_close(_event):
        node.destroy_node()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    fig.canvas.mpl_connect("close_event", _on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
