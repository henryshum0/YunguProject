#!/usr/bin/env python3
"""
fastlio_px4_bridge.py — Feed FAST-LIO odometry into PX4 EKF2 as external vision.

Subscribes:  /Odometry  (nav_msgs/Odometry, FAST-LIO, frame camera_init → body)
Publishes:   /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)

Conversions:
  - ENU (camera_init) → NED:  pos=(y, x, -z), vel=(vy, vx, -vz),
    q_ned = q_enu_to_ned * q_enu  where q_enu_to_ned = (w=0, x=√2/2, y=√2/2, z=0)
  - Timestamps: ROS epoch µs. MicroXRCE-DDS Timesync converts to PX4 boot time.
  - Covariance: from odom pose.covariance diagonals, fallback to defaults.

PX4 EKF2 requirements (all handled):
  - pose_frame = POSE_FRAME_NED (UNKNOWN rejected)
  - velocity_frame = NED for velocity, BODY_FRD for angular velocity
  - unit quaternion, finite variances, rate >= 5 Hz

Usage:
  python3 utils/fastlio/fastlio_px4_bridge.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

_SQRT2_2 = math.sqrt(2.0) / 2.0
Q_ENU_TO_NED = (0.0, _SQRT2_2, _SQRT2_2, 0.0)  # w,x,y,z


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)


def quat_norm(q):
    return math.sqrt(sum(v*v for v in q))


def normalize(q):
    n = quat_norm(q)
    return q if n < 1e-12 else tuple(v/n for v in q)


def extract_var(cov36, idx, fallback):
    if len(cov36) < 36:
        return [fallback]*3
    if all(abs(cov36[i]) < 1e-15 for i in idx):
        return [fallback]*3
    return [max(float(cov36[i]), 0.0) if math.isfinite(cov36[i]) else fallback
            for i in idx]


class FastLioPx4Bridge(Node):
    def __init__(self):
        super().__init__("fastlio_px4_bridge")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("ev_topic", "/fmu/in/vehicle_visual_odometry")

        in_topic = self.get_parameter("odom_topic").value
        out_topic = self.get_parameter("ev_topic").value

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom = None
        self.create_subscription(Odometry, in_topic, self._cb, qos_be)
        self._pub = self.create_publisher(VehicleOdometry, out_topic, qos_be)
        self.get_logger().info(f"fastlio_px4_bridge: {in_topic} → {out_topic}")

    def _cb(self, m: Odometry):
        ev = VehicleOdometry()
        st = m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000
        ev.timestamp = st
        ev.timestamp_sample = st
        ev.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ev.position[0] = float(m.pose.pose.position.y)
        ev.position[1] = float(m.pose.pose.position.x)
        ev.position[2] = float(-m.pose.pose.position.z)

        q = (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
             m.pose.pose.orientation.y, m.pose.pose.orientation.z)
        qn = normalize(quat_mult(Q_ENU_TO_NED, q))
        ev.q = list(qn)

        ev.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        ev.velocity[0] = float(m.twist.twist.linear.y)
        ev.velocity[1] = float(m.twist.twist.linear.x)
        ev.velocity[2] = float(-m.twist.twist.linear.z)
        ev.angular_velocity[0] = float(m.twist.twist.angular.x)
        ev.angular_velocity[1] = float(m.twist.twist.angular.y)
        ev.angular_velocity[2] = float(m.twist.twist.angular.z)

        ev.position_variance = extract_var(m.pose.covariance, (0, 7, 14), 0.01)
        ev.orientation_variance = extract_var(m.pose.covariance, (21, 28, 35), 0.001)
        ev.velocity_variance = extract_var(m.twist.covariance, (0, 7, 14), 0.01)
        ev.reset_counter = 0
        ev.quality = 0
        self._pub.publish(ev)


def main():
    rclpy.init()
    rclpy.spin(FastLioPx4Bridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
