#!/usr/bin/env python3
"""
fly_square.py — Auto-takeoff and fly a small rectangle for FAST-LIO testing.

Sends trajectory setpoints + offboard control mode directly to PX4
(follows the same protocol as offboard_node but in a self-contained script).

Trajectory:  takeoff 2m → hover 3s → 3m forward → 3m right → 3m back → 3m left → land
Total time: ~25 seconds.

Usage:
  source install/setup.bash
  python3 temp/fly_square.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class FlySquare(Node):
    def __init__(self):
        super().__init__("fly_square")

        qos_px4 = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_px4)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_px4)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_px4)

        # Marker for RViz (optional)
        qos_marker = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, "/fly_square/goal", qos_marker)

        self._start_time = self.get_clock().now()
        self._phase = 0          # 0=arm, 1=takeoff, 2=hover, 3-6=square, 7=land
        self._phase_start = self._start_time
        self._hold_pos = (0.0, 0.0, -2.0)  # NED: hover at 2m height
        self._landing = False

        self._timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info("FlySquare ready. Starting auto-flight sequence...")

    def _elapsed(self):
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    def _go_phase(self, p: int):
        self._phase = p
        self._phase_start = self.get_clock().now()
        names = {0: "ARM", 1: "TAKEOFF", 2: "HOVER",
                 3: "FWD", 4: "RIGHT", 5: "BACK", 6: "LEFT", 7: "LAND"}
        self.get_logger().info(f"Phase {p}: {names.get(p, '?')}")

    def _publish_mode(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = False
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self._mode_pub.publish(m)

    def _publish_setpoint(self, x, y, z, yaw=float('nan')):
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        sp.velocity = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yawspeed = float('nan')
        self._traj_pub.publish(sp)

    def _send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self._cmd_pub.publish(msg)

    def _publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "fly_square"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = -float(z)  # NED→ENU for visualization
        m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 1.0
        self._marker_pub.publish(m)

    # ======================================================================
    #  Waypoint sequence (NED coordinates, Z negative = up)
    # ======================================================================
    # Waypoints for the square (relative to takeoff point at 0,0)
    SQUARE = [
        (3.0, 0.0, -2.0),   # 3m forward (NED: +X is north)
        (3.0, 3.0, -2.0),   # 3m right (NED: +Y is east)
        (0.0, 3.0, -2.0),   # 3m back
        (0.0, 0.0, -2.0),   # return to start
    ]

    def _tick(self):
        self._publish_mode()  # always stream offboard mode

        elapsed = self._elapsed()

        # ---- Phase 0: ARM (first 0.5s) --------------------------------
        if self._phase == 0:
            self._publish_setpoint(0, 0, 0)
            if elapsed < 0.3:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if elapsed > 3.0:
                self._go_phase(1)

        # ---- Phase 1: TAKEOFF (climb to 2m) ---------------------------
        elif self._phase == 1:
            self._publish_setpoint(0, 0, -2.0)  # NED: z=-2m is 2m up
            if elapsed > 6.0:
                self._go_phase(2)

        # ---- Phase 2: HOVER (stabilize) --------------------------------
        elif self._phase == 2:
            self._publish_setpoint(0, 0, -2.0)
            self._publish_marker(0, 0, 2.0)
            if elapsed > 3.0:
                self._go_phase(3)

        # ---- Phase 3-6: Fly the square --------------------------------
        elif 3 <= self._phase <= 6:
            wp = self.SQUARE[self._phase - 3]
            self._publish_setpoint(*wp)
            self._publish_marker(*wp)
            if elapsed > 4.0:  # 4 seconds per leg
                if self._phase < 6:
                    self._go_phase(self._phase + 1)
                else:
                    self._go_phase(7)

        # ---- Phase 7: LAND ---------------------------------------------
        elif self._phase == 7:
            self._publish_setpoint(0, 0, -0.5)  # descend close to ground
            if elapsed > 5.0:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.get_logger().info("Flight complete. Disarming...")
                self._timer.cancel()
                raise SystemExit(0)


def main():
    rclpy.init()  # use_sim_time passed via --ros-args on command line
    try:
        rclpy.spin(FlySquare())
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
