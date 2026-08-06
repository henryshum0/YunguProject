#!/usr/bin/env python3
"""
fastlio_px4_bridge.py — Bridge FAST-LIO odometry into PX4 EKF2 external vision.

Subscribes:
  /Odometry  (nav_msgs/Odometry)   — FAST-LIO output, frame camera_init → body

Publishes:
  /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)  — for PX4 EKF2

Coordinate conversion (ENU camera_init → NED):
  Position:  (x, y, z)_ned = (y, x, -z)_enu
  Velocity:  (vx, vy, vz)_ned = (vy, vx, -vz)_enu
  Attitude:  q_ned = q_enu_to_ned * q_enu
      where q_enu_to_ned = (w=0, x=√2/2, y=√2/2, z=0) — 180° about (1,1,0)

  Angular velocity stays in body frame (BODY_FRD).

Timestamp:
  MicroXRCE-DDS Timesync auto-converts ROS epoch µs → PX4 boot µs,
  so we just fill timestamp/timestamp_sample from the incoming header.

Covariance:
  Extracted from nav_msgs/Odometry covariance diagonals.
  Falls back to sensible defaults when covariance is all zeros.

PX4 EKF2 requirements enforced:
  - pose_frame = POSE_FRAME_NED (1)   (UNKNOWN is rejected)
  - velocity_frame = NED for velocity, BODY_FRD for angular velocity
  - Unit quaternion strictly validated
  - All variance arrays must be finite
  - Publish rate >= 5 Hz (EV_MAX_INTERVAL = 200 ms)

Usage:
  ros2 run <package> fastlio_px4_bridge.py --ros-args -p odom_topic:=/Odometry
"""

import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


# ---------------------------------------------------------------------------
# ENU → NED rotation quaternion
# 180° about axis (1/√2, 1/√2, 0) → (w=0, x=√2/2, y=√2/2, z=0)
# ---------------------------------------------------------------------------
_SQRT2_2 = math.sqrt(2.0) / 2.0  # ≈ 0.70710678
Q_ENU_TO_NED = (0.0, _SQRT2_2, _SQRT2_2, 0.0)  # (w, x, y, z) — Hamiltonian convention


def _quat_multiply(q1, q2):
    """Hamiltonian quaternion multiplication: q1 * q2.

    Args:
        q1, q2: (w, x, y, z) tuples.
    Returns:
        (w, x, y, z) tuple.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_norm(q):
    """Euclidean norm of a quaternion."""
    return math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)


def _normalize_quat(q):
    """Normalize quaternion to unit length. Returns identity if input is degenerate."""
    n = _quat_norm(q)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _extract_variance(covariance_36, indices, fallback):
    """Extract three variance values from a 36-element covariance row-major array.

    Args:
        covariance_36: list[float] of length 36.
        indices: tuple of 3 zero-based indices.
        fallback: fallback value if covariance is all zeros.
    Returns:
        list[float] of length 3.
    """
    if len(covariance_36) < 36:
        return [fallback] * 3
    # If all zero, use fallback
    if all(abs(covariance_36[i]) < 1e-15 for i in indices):
        return [fallback] * 3
    vals = [float(covariance_36[i]) for i in indices]
    # Ensure non-negative, finite
    return [max(v, 0.0) if math.isfinite(v) else fallback for v in vals]


class FastLioPx4Bridge(Node):
    """Bridge node: nav_msgs/Odometry → px4_msgs/VehicleOdometry (ENU→NED)."""

    def __init__(self):
        super().__init__("fastlio_px4_bridge")

        # --- Parameters ----------------------------------------------------
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("ev_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("republish_odom_topic", "")  # optional backward compat
        self.declare_parameter("default_pos_var", 0.01)      # m², ≈ 10 cm std
        self.declare_parameter("default_ori_var", 0.001)     # rad², ≈ 1.8° std
        self.declare_parameter("default_vel_var", 0.01)      # (m/s)²
        self.declare_parameter("publish_rate", 100.0)        # Hz, max output rate
        self.declare_parameter("diagnostic_log_rate", 1.0)   # Hz, status log rate

        odom_topic = self.get_parameter("odom_topic").value
        ev_topic = self.get_parameter("ev_topic").value
        repub_topic = self.get_parameter("republish_odom_topic").value
        self._default_pos_var = self.get_parameter("default_pos_var").value
        self._default_ori_var = self.get_parameter("default_ori_var").value
        self._default_vel_var = self.get_parameter("default_vel_var").value

        # --- QoS -----------------------------------------------------------
        # FAST-LIO publishes with default ROS QoS (reliable, keep_last=20).
        # PX4 /fmu/in topics need best_effort to match the MicroXRCE-DDS agent.
        qos_input = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_px4 = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscriber ----------------------------------------------------
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_input,
        )

        # --- Publisher (PX4 EKF2) ------------------------------------------
        self._ev_pub = self.create_publisher(VehicleOdometry, ev_topic, qos_px4)

        # --- Optional backward-compat odom republisher ---------------------
        self._repub = None
        if repub_topic:
            self._repub = self.create_publisher(Odometry, repub_topic, qos_input)

        # --- State ---------------------------------------------------------
        self._latest_odom: Optional[Odometry] = None
        self._reset_counter: int = 0
        self._last_frame_id: str = ""
        self._msg_count: int = 0
        self._last_log_time = self.get_clock().now()

        # --- Timer (throttle output rate) ----------------------------------
        period_s = 1.0 / max(self.get_parameter("publish_rate").value, 1.0)
        self._timer = self.create_timer(period_s, self._publish_ev)

        # --- Diagnostic timer ----------------------------------------------
        log_period_s = 1.0 / max(self.get_parameter("diagnostic_log_rate").value, 0.1)
        self._log_timer = self.create_timer(log_period_s, self._diagnostic_log)

        self.get_logger().info(
            f"fastlio_px4_bridge started:\n"
            f"  subscribing:  {odom_topic}\n"
            f"  publishing:   {ev_topic}\n"
            f"  republishing: {repub_topic or '(disabled)'}\n"
            f"  convert ENU (camera_init) → NED"
        )

    # -----------------------------------------------------------------------
    #  Callback — cache latest odom
    # -----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

        # Detect frame_id change → increment reset counter
        if msg.header.frame_id != self._last_frame_id:
            if self._last_frame_id:
                self._reset_counter += 1
                self.get_logger().warn(
                    f"frame_id changed: '{self._last_frame_id}' → "
                    f"'{msg.header.frame_id}' — reset_counter now "
                    f"{self._reset_counter}"
                )
            self._last_frame_id = msg.header.frame_id

    # -----------------------------------------------------------------------
    #  Timer — convert and publish
    # -----------------------------------------------------------------------
    def _publish_ev(self):
        odom = self._latest_odom
        if odom is None:
            return  # no data yet

        ev = VehicleOdometry()

        # -- Timestamp (ROS epoch µs → bridge converts to PX4 boot µs) ------
        stamp_us = odom.header.stamp.sec * 1_000_000 + odom.header.stamp.nanosec // 1000
        ev.timestamp = stamp_us
        ev.timestamp_sample = stamp_us

        # -- Pose frame: NED ------------------------------------------------
        ev.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # -- Position: ENU → NED  ((x_enu, y_enu, z_enu) → (y, x, -z)) -----
        ev.position[0] = float(odom.pose.pose.position.y)
        ev.position[1] = float(odom.pose.pose.position.x)
        ev.position[2] = float(-odom.pose.pose.position.z)

        # -- Orientation: q_body_in_ned = q_enu_to_ned * q_body_in_enu ------
        # IMPORTANT: this assumes camera_init is approximately ENU-aligned.
        # A constant yaw offset is handled by EKF2's auto-yaw-align (EKF2_EV_CTRL bit 3).
        q_enu = (
            float(odom.pose.pose.orientation.w),
            float(odom.pose.pose.orientation.x),
            float(odom.pose.pose.orientation.y),
            float(odom.pose.pose.orientation.z),
        )
        q_ned = _quat_multiply(Q_ENU_TO_NED, q_enu)
        q_ned = _normalize_quat(q_ned)  # ensure unit length
        ev.q[0] = q_ned[0]  # w
        ev.q[1] = q_ned[1]  # x
        ev.q[2] = q_ned[2]  # y
        ev.q[3] = q_ned[3]  # z

        # -- Velocity frame: NED --------------------------------------------
        ev.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # -- Linear velocity: ENU → NED  ((vx, vy, vz) → (vy, vx, -vz)) ----
        ev.velocity[0] = float(odom.twist.twist.linear.y)
        ev.velocity[1] = float(odom.twist.twist.linear.x)
        ev.velocity[2] = float(-odom.twist.twist.linear.z)

        # -- Angular velocity: stays in body frame, published as BODY_FRD ---
        ev.angular_velocity[0] = float(odom.twist.twist.angular.x)
        ev.angular_velocity[1] = float(odom.twist.twist.angular.y)
        ev.angular_velocity[2] = float(odom.twist.twist.angular.z)

        # -- Variances (REQUIRED: must be finite when EKF2_EV_NOISE_MD=0) ---
        # Row-major 6x6 covariance: indices 0,7,14 = pos; 21,28,35 = orientation
        ev.position_variance = _extract_variance(
            odom.pose.covariance,
            (0, 7, 14),
            self._default_pos_var,
        )
        ev.orientation_variance = _extract_variance(
            odom.pose.covariance,
            (21, 28, 35),
            self._default_ori_var,
        )
        ev.velocity_variance = _extract_variance(
            odom.twist.covariance,
            (0, 7, 14),
            self._default_vel_var,
        )

        # -- Reset counter --------------------------------------------------
        ev.reset_counter = self._reset_counter & 0xFF  # uint8

        # -- Quality (0 = unused when EKF2_EV_QMIN=0) -----------------------
        ev.quality = 0

        # -- Publish to PX4 ------------------------------------------------
        self._ev_pub.publish(ev)
        self._msg_count += 1

        # -- Optional backward-compat odom republish -----------------------
        if self._repub is not None:
            self._repub.publish(odom)

    # -----------------------------------------------------------------------
    #  Diagnostic log
    # -----------------------------------------------------------------------
    def _diagnostic_log(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds * 1e-9
        if elapsed <= 0.0:
            return
        rate = self._msg_count / elapsed if self._msg_count > 0 else 0.0
        odom = self._latest_odom
        if odom is not None:
            self.get_logger().info(
                f"EV bridge: {self._msg_count} msgs, "
                f"rate={rate:.1f} Hz, "
                f"pos=({odom.pose.pose.position.x:.2f}, "
                f"{odom.pose.pose.position.y:.2f}, "
                f"{odom.pose.pose.position.z:.2f})",
                throttle_duration_sec=10.0,
            )
        self._msg_count = 0
        self._last_log_time = now


# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = FastLioPx4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
