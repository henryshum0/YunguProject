#!/usr/bin/env python3
# Copyright 2026
#
# cmd_record: goal-triggered recorder for SUPER's command trajectory plus the
# real drone odometry.
#
# Behaviour
#   - Nothing is recorded until a goal is clicked on /goal_pose.
#   - On each goal message:
#       1. if not recording      -> start a new recording segment
#       2. if already recording  -> stop the previous segment (save its CSV)
#          and start a new segment for the new goal
#   - A segment is stopped and saved when the commanded-trajectory publish rate
#     drops below min_cmd_rate (default 10 Hz), i.e. the planner stopped.
#   - Each CSV row carries the current goal position, so goal + trajectory
#     (position, velocity, acceleration, attitude, body rate, yaw command,
#     yaw-rate command) + the real drone yaw and odometry (position/velocity/
#     body rate) live in one file. The yaw command (pos_cmd.yaw) is wrapped
#     into [-pi, pi] so it is directly comparable with the real drone yaw.
#   - CSVs are written to <project>/cmd_log/ (override with log_dir=).
#   - A live matplotlib window shows cmd vs real odometry per axis (X/Y/Z rows;
#     Position/Velocity/Accel/Attitude/Body-rate columns) with a sliding window
#     (window_sec), updated in realtime. The Body-rate cells show the commanded
#     vs the real (odom) body rate; the Z-row Attitude/Body-rate cells
#     additionally show the yaw command, the yaw-rate command and the real
#     drone yaw.

import csv
import math
import os
import signal
import threading
import time as _time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

# --- Optional matplotlib live plotting ------------------------------------
try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    MPL_OK = True
except Exception:  # pragma: no cover - matplotlib may not be installed
    MPL_OK = False


CSV_HEADER = [
    "t", "gx", "gy", "gz",
    "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az",
    "roll", "pitch", "yaw", "wx", "wy", "wz", "owx", "owy", "owz",
    "yaw_cmd", "yaw_dot_cmd", "oyaw",
    "opx", "opy", "opz", "ovx", "ovy", "ovz",
]

# Live-plot grid: rows = axes (X/Y/Z), cols = quantities.
# (name, cmd pos, odom pos, cmd vel, odom vel, acc, attitude, body rate, goal)
AXIS_ROWS = [
    ("X", "px", "opx", "vx", "ovx", "ax", "roll", "wx", "gx"),
    ("Y", "py", "opy", "vy", "ovy", "ay", "pitch", "wy", "gy"),
    ("Z", "pz", "opz", "vz", "ovz", "az", "yaw", "wz", "gz"),
]
COL_NAMES = ["Position [m]", "Velocity [m/s]", "Accel [m/s^2]",
             "Attitude [rad]", "Body rate [rad/s]"]


def _cell_specs(c, axis):
    """(field, label, color, linestyle) tuples plotted in grid column c."""
    _n, p, op, v, ov, a, att, br, _g = axis
    if c == 0:
        return [(p, "cmd", "tab:blue", "-"), (op, "odom", "tab:red", "--")]
    if c == 1:
        return [(v, "cmd", "tab:blue", "-"), (ov, "odom", "tab:red", "--")]
    if c == 2:
        return [(a, "cmd", "tab:blue", "-")]
    if c == 3:
        # Attitude: commanded roll/pitch/yaw; on Z also plot the yaw command
        # and the real drone yaw (from odometry).
        specs = [(att, "cmd", "tab:blue", "-")]
        if axis[0] == "Z":
            specs.append(("yaw_cmd", "yaw cmd", "tab:green", "-"))
            specs.append(("oyaw", "odom yaw", "tab:red", "--"))
        return specs
    # Body rate: commanded vs real (odom) body rates; on Z also plot the
    # yaw-rate command.
    _br_map = {"wx": "owx", "wy": "owy", "wz": "owz"}
    specs = [(br, "cmd", "tab:blue", "-"),
             (_br_map[br], "odom", "tab:red", "--")]
    if axis[0] == "Z":
        specs.append(("yaw_dot_cmd", "yaw_dot cmd", "tab:green", "-"))
    return specs


class CmdRecordNode(Node):
    def __init__(self):
        super().__init__("cmd_record")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_topic", "/planning/pos_cmd")
        self.declare_parameter("odom_topic", "/lidar_slam/odom")
        self.declare_parameter("log_dir", "")          # empty -> <project>/cmd_log
        self.declare_parameter("min_cmd_rate", 10.0)   # Hz; stop below this rate
        self.declare_parameter("viz_en", True)
        self.declare_parameter("window_sec", 20.0)     # sliding window [s]
        self.declare_parameter("plot_rate", 10.0)      # live plot refresh [Hz]
        self.declare_parameter("use_header_stamp", True)

        goal_topic = self.get_parameter("goal_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.log_dir = self.get_parameter("log_dir").value
        self.min_cmd_rate = float(self.get_parameter("min_cmd_rate").value)
        self.viz_en = self._to_bool(self.get_parameter("viz_en").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.plot_rate = float(self.get_parameter("plot_rate").value)
        self.use_header_stamp = self._to_bool(self.get_parameter("use_header_stamp").value)

        self.get_logger().info(
            f"cmd_record ready: goal={goal_topic} cmd={cmd_topic} odom={odom_topic} "
            f"(min_cmd_rate={self.min_cmd_rate:g} Hz)")

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._recording = False
        self._seg = []             # rows of the current segment
        self._written = 0          # rows already flushed to the CSV
        self._goal = None          # current goal dict {idx, x, y, z}
        self._goal_idx = 0
        self._goal_t = None        # time the current goal was received
        self._seg_start = None
        self._last_cmd = None      # node-clock time of the last cmd
        self._cmd_times = deque(maxlen=500)
        self._last_odom = None     # latest nav_msgs/Odometry
        self._last_odom_yaw = None # drone yaw [rad] from the latest odom quaternion
        self._last_odom_ang = None # (wx, wy, wz) body rate from the latest odom twist
        self._odom_count = 0
        self._lock = threading.Lock()

        self._csv_path = ""
        self._csv_writer = None
        self._csv_handle = None

        # plot ring buffers (bounded): per grid cell, one deque per plotted line
        self._pt = deque(maxlen=20000)
        self._pd = [[[deque(maxlen=20000) for _ in _cell_specs(c, axis)]
                     for c in range(5)] for axis in AXIS_ROWS]
        self._goal_marker = None   # (t, idx) vertical marker on the plot

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        # SUPER publishes /planning/pos_cmd with best_effort QoS; a reliable
        # subscription is INCOMPATIBLE with a best_effort publisher, so DDS
        # never connects and no cmd arrives. best_effort is compatible with
        # both best_effort (SUPER cmd) and reliable (RViz goal, odom) publishers.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, qos)
        self.create_subscription(PositionCommand, cmd_topic, self._cmd_cb, qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)

        # periodic flush (survives a crash) + rate watchdog
        self.create_timer(5.0, self._periodic_flush)
        self.create_timer(0.5, self._watchdog)

        # ------------------------------------------------------------------
        # Live plot
        # ------------------------------------------------------------------
        self.fig = None
        self.axs = None
        self.lines = None        # 3x5 grid of line lists (built in _setup_plot)
        self.goal_lines = None   # 3x1 goal position lines (built in _setup_plot)
        self._goal_artists = []
        if self.viz_en:
            if MPL_OK:
                try:
                    self._start_plot()
                except Exception as exc:  # pragma: no cover - no display etc.
                    self.fig = None
                    self.get_logger().warn(
                        f"Live visualization unavailable ({exc}); recording to CSV only")
            else:
                self.get_logger().warn(
                    "matplotlib not available - recording to CSV only")

    # ======================================================================
    #  Callbacks
    # ======================================================================
    def _goal_cb(self, msg):
        t = self._stamp(msg.header.stamp)
        with self._lock:
            self._goal_idx += 1
            goal = {
                "idx": self._goal_idx,
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            }
            if self._recording:
                self._stop_recording_locked(f"new goal #{self._goal_idx}")
            self._goal = goal
            self._goal_t = t
            self._start_recording_locked()
        self.get_logger().info(
            f"Goal #{goal['idx']} @ t={t:.3f}s pos=({goal['x']:.2f}, {goal['y']:.2f}, "
            f"{goal['z']:.2f}) -> recording")

    def _cmd_cb(self, msg):
        with self._lock:
            if not self._recording:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            t = self._stamp(msg.header.stamp)
            self._last_cmd = now
            self._cmd_times.append(now)

            g = self._goal
            od = self._last_odom
            row = {
                "t": t,
                "gx": g["x"], "gy": g["y"], "gz": g["z"],
                "px": msg.position.x, "py": msg.position.y, "pz": msg.position.z,
                "vx": msg.velocity.x, "vy": msg.velocity.y, "vz": msg.velocity.z,
                "ax": msg.acceleration.x, "ay": msg.acceleration.y, "az": msg.acceleration.z,
                "roll": msg.attitude.x, "pitch": msg.attitude.y, "yaw": msg.attitude.z,
                "wx": msg.angular_velocity.x, "wy": msg.angular_velocity.y,
                "wz": msg.angular_velocity.z,
                # real (odom) body rate from the latest /lidar_slam/odom twist
                "owx": self._last_odom_ang[0] if self._last_odom_ang is not None
                       else float("nan"),
                "owy": self._last_odom_ang[1] if self._last_odom_ang is not None
                       else float("nan"),
                "owz": self._last_odom_ang[2] if self._last_odom_ang is not None
                       else float("nan"),
                # SUPER's yaw command is an unwrapped (continuous) angle that
                # can exceed [-pi, pi]; wrap it into [-pi, pi] so it is directly
                # comparable with the real drone yaw (oyaw).
                "yaw_cmd": math.remainder(msg.yaw, 2.0 * math.pi),
                "yaw_dot_cmd": msg.yaw_dot,
                "oyaw": self._last_odom_yaw if self._last_odom_yaw is not None
                        else float("nan"),
            }
            if od is not None:
                row["opx"] = od.pose.pose.position.x
                row["opy"] = od.pose.pose.position.y
                row["opz"] = od.pose.pose.position.z
                row["ovx"] = od.twist.twist.linear.x
                row["ovy"] = od.twist.twist.linear.y
                row["ovz"] = od.twist.twist.linear.z
            else:
                for k in ("opx", "opy", "opz", "ovx", "ovy", "ovz"):
                    row[k] = float("nan")
            self._seg.append(row)

            # live-plot ring buffers (per grid cell)
            self._pt.append(t)
            for r, axis in enumerate(AXIS_ROWS):
                for c in range(5):
                    for ch, (field, *_rest) in zip(self._pd[r][c],
                                                   _cell_specs(c, axis)):
                        ch.append(row[field])

    def _odom_cb(self, msg):
        with self._lock:
            self._last_odom = msg
            self._last_odom_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
            av = msg.twist.twist.angular
            self._last_odom_ang = (av.x, av.y, av.z)
            self._odom_count += 1

    # ======================================================================
    #  Recording control (call while holding self._lock)
    # ======================================================================
    def _start_recording_locked(self):
        if self._recording:
            return
        self._recording = True
        self._seg = []
        self._written = 0
        self._seg_start = self.get_clock().now().nanoseconds * 1e-9
        self._last_cmd = None
        self._cmd_times.clear()
        self._goal_marker = ((self._goal_t, self._goal_idx)
                             if self._goal_t is not None else None)
        # clear live-plot buffers for the new segment
        self._pt.clear()
        for row_cells in self._pd:
            for cell in row_cells:
                for ch in cell:
                    ch.clear()
        self._open_csv()
        self.get_logger().info(f"Recording started -> {self._csv_path}")

    def _stop_recording_locked(self, reason=""):
        if not self._recording:
            return
        self._recording = False
        self._flush_csv_locked()
        self._close_csv()
        self.get_logger().info(
            f"Recording stopped ({reason or 'end'}) -> saved {self._csv_path} "
            f"[{len(self._seg)} rows, {self._odom_count} odom msgs received]")
        self._csv_path = ""

    def _watchdog(self):
        with self._lock:
            if not self._recording:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._seg_start < 1.0:      # warmup - don't judge yet
                return
            if self._last_cmd is None:           # no cmd received at all
                return
            recent = sum(1 for ct in self._cmd_times if now - ct <= 1.0)
            if recent < self.min_cmd_rate:
                self._stop_recording_locked(
                    f"cmd rate {recent:.1f} Hz < {self.min_cmd_rate:g} Hz")
            elif now - self._last_cmd > 2.0:
                self._stop_recording_locked("no cmd data")

    # ======================================================================
    #  CSV
    # ======================================================================
    def _open_csv(self):
        log_dir = self.log_dir or self._default_log_dir()
        os.makedirs(log_dir, exist_ok=True)
        ts = _time.strftime("%Y%m%d_%H%M%S")
        name = "goal_%03d_%s.csv" % (self._goal_idx, ts)
        self._csv_path = os.path.join(log_dir, name)
        self._csv_handle = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_handle)
        self._csv_writer.writerow(CSV_HEADER)
        self._csv_handle.flush()

    def _flush_csv_locked(self):
        if self._csv_writer is None:
            return
        while self._written < len(self._seg):
            r = self._seg[self._written]
            self._csv_writer.writerow([
                "%.9f" % r["t"],
                "%.4f" % r["gx"], "%.4f" % r["gy"], "%.4f" % r["gz"],
                "%.6f" % r["px"], "%.6f" % r["py"], "%.6f" % r["pz"],
                "%.6f" % r["vx"], "%.6f" % r["vy"], "%.6f" % r["vz"],
                "%.6f" % r["ax"], "%.6f" % r["ay"], "%.6f" % r["az"],
                "%.6f" % r["roll"], "%.6f" % r["pitch"], "%.6f" % r["yaw"],
                "%.6f" % r["wx"], "%.6f" % r["wy"], "%.6f" % r["wz"],
                "%.6f" % r["owx"], "%.6f" % r["owy"], "%.6f" % r["owz"],
                "%.6f" % r["yaw_cmd"], "%.6f" % r["yaw_dot_cmd"],
                "%.6f" % r["oyaw"],
                "%.6f" % r["opx"], "%.6f" % r["opy"], "%.6f" % r["opz"],
                "%.6f" % r["ovx"], "%.6f" % r["ovy"], "%.6f" % r["ovz"],
            ])
            self._written += 1
        self._csv_handle.flush()

    def _periodic_flush(self):
        with self._lock:
            self._flush_csv_locked()

    def _close_csv(self):
        if self._csv_handle is not None:
            self._csv_handle.close()
            self._csv_handle = None
            self._csv_writer = None

    # ======================================================================
    #  Live plot
    # ======================================================================
    def _setup_plot(self):
        """Build the 3x5 grid: rows X/Y/Z, cols Position/Velocity/Accel/
        Attitude/Body-rate. cmd (solid) vs odom (dashed) on Pos/Vel columns."""
        self.fig, self.axs = plt.subplots(3, 5, figsize=(22, 11), sharex=True)
        self.lines = [[[] for _ in range(5)] for _ in range(3)]
        self.goal_lines = [[None] * 5 for _ in range(3)]
        for r, axis in enumerate(AXIS_ROWS):
            name = axis[0]
            for c in range(5):
                ax = self.axs[r, c]
                ax.set_title(f"{name} {COL_NAMES[c]}", fontsize=9)
                ax.set_xlabel("time [s]", fontsize=8)
                ax.grid(True, alpha=0.4)
                for field, label, color, ls in _cell_specs(c, axis):
                    (ln,) = ax.plot([], [], label=label, color=color,
                                    linestyle=ls, linewidth=1.4)
                    self.lines[r][c].append(ln)
                if c == 0:  # goal position reference line
                    (gl,) = ax.plot([], [], label="goal", color="gray",
                                    linestyle=":", linewidth=1.2)
                    self.goal_lines[r][0] = gl
                if r == 0:
                    ax.legend(loc="upper right", fontsize=6, ncol=3)
            self.axs[r, 0].set_ylabel(name, fontsize=11)
        self.fig.tight_layout()

    def _start_plot(self):
        """Create the figure + data-update routine.

        The GUI is pumped from main() on the main thread (plt.pause), while the
        ROS executor runs on a background thread so command/odometry processing
        (and the rate watchdog) stay timely.
        """
        self._setup_plot()
        plt.ion()

    def _update_plot_data(self):
        """Refresh line data + goal markers (called from main() on the main
        thread before each plt.pause, which performs the actual draw)."""
        if self.fig is None:
            return
        with self._lock:
            t = list(self._pt)
            d = [[[list(ch) for ch in cell] for cell in row]
                 for row in self._pd]
            goal = self._goal
            marker = self._goal_marker
        if len(t) < 2:
            return
        try:
            for r in range(3):
                for c in range(5):
                    ax = self.axs[r, c]
                    for ln, data in zip(self.lines[r][c], d[r][c]):
                        ln.set_data(t, data)
                    ax.relim()
                    ax.autoscale_view()
            if self.window_sec > 0:
                tnow = t[-1]
                for ax in self.axs.ravel():
                    ax.set_xlim(tnow - self.window_sec, tnow)
            # goal position reference lines (constant per segment)
            if goal is not None:
                for r in range(3):
                    gl = self.goal_lines[r][0]
                    if gl is not None:
                        gval = [goal["x"], goal["y"], goal["z"]][r]
                        gl.set_data([t[0], t[-1]], [gval, gval])
            self._clear_goal_artists()
            if marker is not None:
                self._add_goal_marker(*marker)
        except Exception as exc:  # pragma: no cover - window closed etc.
            self.get_logger().warn(f"Plot update failed: {exc}")

    def _clear_goal_artists(self):
        for art in self._goal_artists:
            try:
                art.remove()
            except Exception:  # pragma: no cover - already removed
                pass
        self._goal_artists = []

    def _add_goal_marker(self, t0, idx):
        for ax in self.axs.ravel():
            xmin, xmax = ax.get_xlim()
            if t0 < xmin or t0 > xmax:
                continue
            self._goal_artists.append(
                ax.axvline(t0, color="gray", linestyle="--",
                           linewidth=0.8, alpha=0.7))
        ytop = self.axs[0, 0].get_ylim()[1]
        self._goal_artists.append(
            self.axs[0, 0].text(t0, ytop, f" G{idx}", color="gray",
                                fontsize=8, va="top", ha="left"))

    # ======================================================================
    #  Helpers
    # ======================================================================
    def _stamp(self, stamp):
        if self.use_header_stamp:
            t = stamp.sec + stamp.nanosec * 1e-9
            if t <= 0.0:
                t = self.get_clock().now().nanoseconds * 1e-9
            return t
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _to_bool(v):
        if isinstance(v, bool):
            return v
        return str(v).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _quat_to_yaw(q):
        """Yaw [rad] about Z from a quaternion (w, x, y, z)."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _find_workspace_root():
        """Locate the colcon workspace root (the dir holding src/ and install/)."""
        d = os.path.dirname(os.path.realpath(__file__))
        for _ in range(8):
            if (os.path.isdir(os.path.join(d, "src"))
                    and os.path.isdir(os.path.join(d, "install"))):
                return d
            parent = os.path.dirname(d)
            if parent == d:
                return None
            d = parent
        return None

    def _default_log_dir(self):
        """<project>/cmd_log when running from a source checkout, otherwise the
        installed package's share dir."""
        ws = self._find_workspace_root()
        if ws:
            return os.path.join(ws, "cmd_log")
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("cmd_record"), "cmd_log")
        except Exception:  # pragma: no cover - AMENT_PREFIX_PATH not set
            return os.path.join(os.path.expanduser("~/.ros"))

    def finalize(self):
        with self._lock:
            if self._recording:
                self._stop_recording_locked("shutdown")
        # NOTE: the figure is intentionally NOT closed here. matplotlib's Tk
        # background main-loop thread can SIGABRT during interpreter teardown in
        # WSL; the GUI branch of main() uses os._exit(0) after saving instead.


def main(args=None):
    rclpy.init(args=args)
    node = CmdRecordNode()

    gui = MPL_OK and node.fig is not None
    executor = None
    if gui:
        # ROS executor on a background thread so cmd/odom processing and the
        # rate watchdog stay timely; the GUI is pumped on the main thread.
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        def _spin_quiet():
            try:
                executor.spin()
            except (KeyboardInterrupt,
                    rclpy.executors.ExternalShutdownException):
                pass

        threading.Thread(target=_spin_quiet, daemon=True).start()

        shutdown = threading.Event()
        signal.signal(signal.SIGINT, lambda s, f: shutdown.set())

        interval = 1.0 / max(node.plot_rate, 0.1)
        try:
            while (not shutdown.is_set()
                   and plt.fignum_exists(node.fig.number)):
                node._update_plot_data()
                plt.pause(interval)
        except Exception as exc:  # pragma: no cover - no display etc.
            node.get_logger().warn(
                f"Live visualization unavailable ({exc}); recording to CSV only")
            shutdown.wait()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    try:
        node.finalize()
    finally:
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if gui:
        # Skip matplotlib/Tk interpreter teardown (can SIGABRT in WSL); all
        # data was already flushed/closed in finalize().
        os._exit(0)


if __name__ == "__main__":
    main()
