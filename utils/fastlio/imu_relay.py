#!/usr/bin/python3
# -*- coding: utf-8 -*-
# NOTE: shebang is /usr/bin/python3 (not `env python3`) on purpose: this system
# has a conda Python 3.14 on PATH that cannot load the cp310 rclpy C extension.
"""imu_relay.py — Monotonicize gz IMU timestamps before FAST-LIO.

gz IMU is bridged to /livox/imu_raw with the sim clock (same source as the
lidar clouds). FAST-LIO aborts on regressing IMU stamps ("cannot store a
negative time point"), and at 250 Hz DDS delivery order can jitter — a
non-monotonic stamp made FAST-LIO diverge ("No Effective Points!" forever
after one bad frame). This relay clamps stamps to a strictly increasing
sequence and logs how often it had to (throttled), so we can see whether
jitter is real or the divergence had another cause.

Usage (started by start_fastlio.sh):
  python3 utils/fastlio/imu_relay.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

TOPIC_IN = "/livox/imu_raw"
TOPIC_OUT = "/livox/imu"


class ImuRelayNode(Node):
    def __init__(self):
        super().__init__("imu_relay")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, TOPIC_IN, self._cb, qos)
        # FAST-LIO subscribes with default QoS (reliable) — reliable output
        # is the cleanest match.
        self._pub = self.create_publisher(Imu, TOPIC_OUT, 10)
        self._last = None      # last published stamp as (sec, nanosec)
        self._clamped = 0      # total clamped messages
        self._cb_count = 0
        self.get_logger().info(f"imu relay: {TOPIC_IN} → {TOPIC_OUT} (monotonic)")

    def _cb(self, msg: Imu):
        self._cb_count += 1
        s = msg.header.stamp
        key = (s.sec, s.nanosec)
        if self._last is not None and key <= self._last:
            # Regressing/duplicate stamp — nudge forward by 1 µs past the
            # last published one.
            self._clamped += 1
            sec, nsec = self._last
            nsec += 1000
            if nsec >= 1_000_000_000:
                sec += 1
                nsec -= 1_000_000_000
            s.sec, s.nanosec = sec, nsec
            key = (s.sec, s.nanosec)
        self._last = key
        if self._clamped and self._cb_count % 500 == 0:
            self.get_logger().warn(
                f"imu relay: {self._clamped}/{self._cb_count} stamps clamped "
                f"(non-monotonic gz IMU)")
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
