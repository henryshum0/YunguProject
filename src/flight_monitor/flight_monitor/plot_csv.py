#!/usr/bin/env python3
# Copyright 2026
#
# Post-hoc visualization of cmd_record CSVs.
#
# Usage:
#   python3 src/cmd_record/cmd_record/plot_csv.py [path.csv] [--save out.png]
#   ros2 run cmd_record plot_csv [path.csv] [--save out.png]
#
# With no path, it plots the most recent goal_*.csv in <project>/cmd_log.
#
# Layout: 3 rows (X / Y / Z axes) x 5 columns (Position, Velocity, Accel,
# Attitude, Body rate). Commanded (solid blue) and real odometry (dashed red)
# are overlaid on the Position, Velocity and Body-rate columns so the tracking
# response can be compared per axis; the goal position is drawn as a dotted gray
# line on the Position column.

import argparse
import csv
import os
import sys

import numpy as np

import matplotlib

# interactive window when a display is available, else headless (PNG via --save)
for backend in ("TkAgg", "Agg"):
    try:
        matplotlib.use(backend)
        import matplotlib.pyplot as plt
        break
    except Exception:  # pragma: no cover
        continue


def _workspace_root():
    d = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    for _ in range(8):
        if (os.path.isdir(os.path.join(d, "src"))
                and os.path.isdir(os.path.join(d, "install"))):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent
    return None


def _newest_csv():
    ws = _workspace_root()
    log_dir = os.path.join(ws, "cmd_log") if ws else None
    if not log_dir or not os.path.isdir(log_dir):
        return None
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir)
             if f.startswith("goal_") and f.endswith(".csv")]
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _load(path):
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    return rows


def _col(rows, name):
    return np.array([float(r[name]) for r in rows])


# rows: (axis name, goal col, cmd pos, odom pos, cmd vel, odom vel, acc, att, body rate)
AXES = [
    ("X", "gx", "px", "opx", "vx", "ovx", "ax", "roll", "wx"),
    ("Y", "gy", "py", "opy", "vy", "ovy", "ay", "pitch", "wy"),
    ("Z", "gz", "pz", "opz", "vz", "ovz", "az", "yaw", "wz"),
]
COLS = ["Position [m]", "Velocity [m/s]", "Accel [m/s^2]",
        "Attitude [rad]", "Body rate [rad/s]"]


def main():
    ap = argparse.ArgumentParser(
        description="Plot a cmd_record CSV: cmd vs real odometry per axis "
                    "(X/Y/Z rows; Position/Velocity/Accel/Attitude/Body-rate cols).")
    ap.add_argument("path", nargs="?", default=None,
                    help="CSV to plot (default: newest goal_*.csv in cmd_log)")
    ap.add_argument("--save", default=None,
                    help="Save the figure to this PNG instead of opening a window")
    ap.add_argument("--start", type=float, default=0.0,
                    help="Start time [s] to show (default 0)")
    ap.add_argument("--end", type=float, default=None,
                    help="End time [s] to show (default: end of data)")
    args = ap.parse_args()

    path = args.path or _newest_csv()
    if not path or not os.path.isfile(path):
        print("No CSV found. Pass a path or record something first "
              "(files live in <project>/cmd_log).", file=sys.stderr)
        sys.exit(1)

    rows = _load(path)
    if not rows:
        print(f"{path}: empty CSV", file=sys.stderr)
        sys.exit(1)

    t = _col(rows, "t")
    t = t - t[0]                     # relative time [s] from the goal

    fig, axs = plt.subplots(3, 5, figsize=(22, 11), sharex=True)

    for r, axis in enumerate(AXES):
        name, gcol, p, op, v, ov, a, att, br = axis
        goal_val = float(rows[0][gcol])
        for c in range(5):
            ax = axs[r, c]
            ax.set_title(f"{name} {COLS[c]}", fontsize=10)
            ax.grid(True, alpha=0.4)
            if c == 0:                 # Position: cmd vs odom vs goal
                ax.plot(t, _col(rows, p), label="cmd", color="tab:blue",
                        linewidth=1.5)
                ax.plot(t, _col(rows, op), label="odom", color="tab:red",
                        linestyle="--", linewidth=1.2)
                ax.axhline(goal_val, color="gray", linestyle=":",
                           linewidth=1.2, label="goal")
            elif c == 1:               # Velocity: cmd vs odom
                ax.plot(t, _col(rows, v), label="cmd", color="tab:blue",
                        linewidth=1.5)
                ax.plot(t, _col(rows, ov), label="odom", color="tab:red",
                        linestyle="--", linewidth=1.2)
            elif c == 2:               # Acceleration: cmd
                ax.plot(t, _col(rows, a), label="cmd", color="tab:blue",
                        linewidth=1.5)
            elif c == 3:               # Attitude: cmd (+ yaw cmd / odom yaw on Z)
                ax.plot(t, _col(rows, att), label="cmd", color="tab:blue",
                        linewidth=1.5)
                if name == "Z" and "yaw_cmd" in rows[0]:
                    ax.plot(t, _col(rows, "yaw_cmd"), label="yaw cmd",
                            color="tab:green", linewidth=1.5)
                if name == "Z" and "oyaw" in rows[0]:
                    ax.plot(t, _col(rows, "oyaw"), label="odom yaw",
                            color="tab:red", linestyle="--", linewidth=1.2)
            else:                      # Body rate: cmd vs odom (+ yaw_dot cmd on Z)
                ax.plot(t, _col(rows, br), label="cmd", color="tab:blue",
                        linewidth=1.5)
                o_br = "o" + br        # e.g. wx -> owx (real body rate)
                if o_br in rows[0]:
                    ax.plot(t, _col(rows, o_br), label="odom", color="tab:red",
                            linestyle="--", linewidth=1.2)
                if name == "Z" and "yaw_dot_cmd" in rows[0]:
                    ax.plot(t, _col(rows, "yaw_dot_cmd"), label="yaw_dot cmd",
                            color="tab:green", linewidth=1.5)
            if r == 0:
                ax.legend(loc="upper right", fontsize=7, ncol=3)

    # axis row labels on the left; shared time axis on the bottom row
    for r, (name, *_rest) in enumerate(AXES):
        axs[r, 0].set_ylabel(name, fontsize=12)
    for c in range(5):
        axs[2, c].set_xlabel("time [s]")

    # optional time-window zoom
    if args.start > 0 or args.end is not None:
        end = args.end if args.end is not None else t[-1]
        for ax in axs.ravel():
            ax.set_xlim(args.start, end)

    fig.suptitle(f"{os.path.basename(path)}  (goal "
                 f"({rows[0]['gx']}, {rows[0]['gy']}, {rows[0]['gz']})  t=0 = goal)",
                 fontsize=12)
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Saved plot -> {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
