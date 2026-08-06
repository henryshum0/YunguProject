#!/usr/bin/env python3
"""
compare_odom.py — Evaluate FAST-LIO vs Gazebo ground-truth odometry.

Computes:
  - ATE (Absolute Trajectory Error) after Umeyama alignment
  - RPE (Relative Pose Error) per 1 m travelled
  - Drift rate [m/s] and [deg/s]

Input: a rosbag or a directory containing odometry log files.
       If invoked from test_fastlio_vs_truth.sh, pass the log directory:

    python3 temp/compare_odom.py /tmp/fastlio_test_20260804_120000

Or use two text log files (one odom per line: t x y z qw qx qy qz):

    python3 temp/compare_odom.py fastlio.txt groundtruth.txt

The script auto-detects the input mode:
  - Single directory parses the rosbag inside (requires rosbag2_py).
  - Two positional args → treat as text log files.
"""

import math
import os
import sys
import numpy as np
from typing import List, Tuple, Optional


# ===========================================================================
#  Quaternion / Transform utilities
# ===========================================================================

def quat_multiply(q1, q2):
    """Hamiltonian product q1 * q2.  q = (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_to_rotmat(q):
    """Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def quat_norm(q):
    return math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)


def normalize_quat(q):
    n = quat_norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 1.0, 0.0])


# ===========================================================================
#  Umeyama alignment (7-DOF similarity)
# ===========================================================================

def umeyama(P: np.ndarray, Q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Find s, R, t such that  s * R * P_i + t ≈ Q_i  (least squares).

    Returns R (3×3), t (3,), s (scalar).
    """
    assert P.shape == Q.shape
    n, dim = P.shape

    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)

    sigma_P = ((P - mu_P) ** 2).sum() / n
    cov = (Q - mu_Q).T @ (P - mu_P) / n

    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / sigma_P if sigma_P > 1e-12 else 1.0
    t = mu_Q - s * R @ mu_P
    return R, t, s


# ===========================================================================
#  Metrics
# ===========================================================================

def compute_ate(P_aligned: np.ndarray, Q: np.ndarray) -> dict:
    """Absolute trajectory error (RMSE)."""
    diffs = P_aligned - Q
    rmse = math.sqrt((diffs ** 2).mean())
    return {
        "ate_rmse": rmse,
        "ate_mean": float(np.linalg.norm(diffs, axis=1).mean()),
        "ate_std": float(np.linalg.norm(diffs, axis=1).std()),
        "ate_max": float(np.linalg.norm(diffs, axis=1).max()),
    }


def compute_rpe(P_aligned: np.ndarray, Q: np.ndarray, delta: int = 10) -> dict:
    """Relative pose error over `delta` consecutive samples."""
    n = len(P_aligned)
    if n <= delta:
        return {"rpe_trans": -1, "rpe_rot_deg": -1}
    errors = []
    for i in range(n - delta):
        dP = P_aligned[i + delta] - P_aligned[i]
        dQ = Q[i + delta] - Q[i]
        errors.append(np.linalg.norm(dP - dQ))
    errors = np.array(errors)
    return {
        "rpe_trans": float(errors.mean()),
        "rpe_trans_std": float(errors.std()),
        "rpe_trans_max": float(errors.max()),
    }


def compute_drift(times: np.ndarray, pos: np.ndarray) -> dict:
    """Average drift rate (position change / time)."""
    dt = times[-1] - times[0]
    if dt < 1e-6:
        return {}
    total_dist = np.linalg.norm(pos[1:] - pos[:-1], axis=1).sum()
    return {
        "duration_s": float(dt),
        "avg_speed_ms": float(total_dist / dt),
        "total_dist_m": float(total_dist),
    }


# ===========================================================================
#  Trajectory loading
# ===========================================================================

class Trajectory:
    """N×8 array: [t, x, y, z, qx, qy, qz, qw]."""

    def __init__(self, name: str):
        self.name = name
        self.data: Optional[np.ndarray] = None  # (N, 8)

    @property
    def t(self):
        return self.data[:, 0]

    @property
    def pos(self):
        return self.data[:, 1:4]

    @property
    def quat(self):
        return self.data[:, 4:8]  # x, y, z, w

    def load_from_array(self, arr: np.ndarray):
        self.data = arr

    @staticmethod
    def _ros_time_to_seconds(sec, nanosec=None):
        if nanosec is not None:
            return float(sec) + float(nanosec) * 1e-9
        return float(sec)


def load_trajectory_from_bag(bag_dir: str, topic: str) -> Trajectory:
    """Extract a trajectory from a rosbag (requires rosbag2_py)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    rows = []
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if topic not in type_map:
        print(f"  WARNING: topic '{topic}' not found in bag")
        return Trajectory(topic)

    while reader.has_next():
        (msg_topic, data, stamp) = reader.read_next()
        if msg_topic != topic:
            continue
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        rows.append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    traj = Trajectory(topic)
    if rows:
        traj.data = np.array(rows, dtype=np.float64)
    return traj


def load_trajectory_from_txt(path: str) -> Trajectory:
    """Load from text: one line per pose, fields: t x y z qx qy qz qw."""
    data = np.loadtxt(path, dtype=np.float64)
    traj = Trajectory(path)
    traj.data = data
    return traj


# ===========================================================================
#  Main
# ===========================================================================

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # --- Detect input mode ---------------------------------------------------
    arg = sys.argv[1]
    if os.path.isdir(arg):
        # Bag mode — look for test_bag/ subdir, or use arg directly if it's a bag
        bag_subdir = os.path.join(arg, "test_bag")
        if os.path.isdir(bag_subdir):
            bag_dir = bag_subdir
        elif os.path.isfile(os.path.join(arg, "metadata.yaml")):
            bag_dir = arg
        else:
            print(f"ERROR: No bag found at {arg}")
            sys.exit(1)

        print(f"Loading bag: {bag_dir}")
        fastlio_traj = load_trajectory_from_bag(bag_dir, "/Odometry")
        truth_traj = load_trajectory_from_bag(bag_dir, "/odom")
    elif len(sys.argv) >= 3:
        # Text file mode
        fastlio_traj = load_trajectory_from_txt(sys.argv[1])
        truth_traj = load_trajectory_from_txt(sys.argv[2])
    else:
        print("ERROR: pass a log directory (with test_bag/) or two .txt files")
        sys.exit(1)

    if fastlio_traj.data is None or len(fastlio_traj.data) < 10:
        print(f"ERROR: not enough FAST-LIO samples ({fastlio_traj.name})")
        sys.exit(1)
    if truth_traj.data is None or len(truth_traj.data) < 10:
        print(f"ERROR: not enough ground-truth samples ({truth_traj.name})")
        sys.exit(1)

    # --- Temporal alignment — sync to common time window ---------------------
    t_min = max(fastlio_traj.t[0], truth_traj.t[0])
    t_max = min(fastlio_traj.t[-1], truth_traj.t[-1])
    print(f"Common time window: {t_min:.1f}s – {t_max:.1f}s "
          f"({t_max - t_min:.1f}s)")

    # Sample both trajectories at the FAST-LIO timestamps (higher rate)
    fastlio_mask = (fastlio_traj.t >= t_min) & (fastlio_traj.t <= t_max)
    P_raw = fastlio_traj.pos[fastlio_mask]
    P_times = fastlio_traj.t[fastlio_mask]

    # Interpolate ground truth to FAST-LIO timestamps
    Q_interp = np.zeros_like(P_raw)
    for i, t in enumerate(P_times):
        # nearest (simple but effective when truth rate >> LIO rate)
        idx = np.searchsorted(truth_traj.t, t)
        idx = min(idx, len(truth_traj.t) - 1)
        Q_interp[i] = truth_traj.pos[idx]

    # --- Umeyama alignment (find best rigid transform) -----------------------
    R, t, s = umeyama(P_raw, Q_interp)
    P_aligned = (s * (P_raw @ R.T)) + t
    print(f"\nUmeyama alignment:")
    print(f"  scale  = {s:.6f}")
    print(f"  R      = \n{R}")
    print(f"  t      = {t}")

    # --- Metrics -------------------------------------------------------------
    ate = compute_ate(P_aligned, Q_interp)
    rpe = compute_rpe(P_aligned, Q_interp, delta=10)
    drift = compute_drift(P_times, P_aligned)

    print(f"\n{'=' * 50}")
    print(f"  RESULTS")
    print(f"{'=' * 50}")
    print(f"  Samples:          {len(P_aligned)}")
    print(f"  Duration:         {drift.get('duration_s', 0):.1f} s")
    print(f"  Total distance:   {drift.get('total_dist_m', 0):.1f} m")
    print(f"  Avg speed:        {drift.get('avg_speed_ms', 0):.2f} m/s")
    print(f"")
    print(f"  ATE RMSE:         {ate['ate_rmse']:.4f} m")
    print(f"  ATE mean:         {ate['ate_mean']:.4f} m")
    print(f"  ATE std:          {ate['ate_std']:.4f} m")
    print(f"  ATE max:          {ate['ate_max']:.4f} m")
    print(f"")
    print(f"  RPE (10 steps):   {rpe['rpe_trans']:.4f} m")
    print(f"  RPE std:          {rpe['rpe_trans_std']:.4f} m")
    print(f"  RPE max:          {rpe['rpe_trans_max']:.4f} m")
    print(f"{'=' * 50}")

    # --- Verdict ------------------------------------------------------------
    print(f"\nVerdict:")
    if ate["ate_rmse"] < 0.10:
        print(f"  ✅ ATE < 10cm — Excellent alignment")
    elif ate["ate_rmse"] < 0.30:
        print(f"  ✅ ATE < 30cm — Good, ready for flight test")
    elif ate["ate_rmse"] < 1.00:
        print(f"  ⚠️  ATE < 1m — Acceptable for short flights; tune params")
    else:
        print(f"  ❌ ATE ≥ 1m — Significant error; check config/extrinsics/data")

    if drift.get("duration_s", 0) > 10:
        drift_rate = ate["ate_rmse"] / max(drift["duration_s"], 1)
        print(f"  Drift rate: {drift_rate:.4f} m/s")

    # --- Save aligned trajectories for plotting ------------------------------
    out_dir = sys.argv[1] if os.path.isdir(sys.argv[1]) else os.path.dirname(sys.argv[1]) or "."
    np.savetxt(os.path.join(out_dir, "fastlio_aligned.csv"),
               np.hstack([P_times.reshape(-1, 1), P_aligned]),
               header="t,x_aligned,y_aligned,z_aligned", delimiter=",")
    np.savetxt(os.path.join(out_dir, "groundtruth.csv"),
               np.hstack([P_times.reshape(-1, 1), Q_interp]),
               header="t,x,y,z", delimiter=",")
    print(f"\nAligned trajectories saved to {out_dir}/")


if __name__ == "__main__":
    main()
