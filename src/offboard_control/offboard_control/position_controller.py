#!/usr/bin/env python3
"""
PX4 offboard position controller.

Sends position setpoints to a PX4-autopilot in offboard mode.
- Arms the vehicle
- Switches to offboard mode
- Streams position setpoints at high rate
- Sends a disarm command on shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)
import numpy as np


class PositionController(Node):
    """Offboard position controller for PX4."""

    def __init__(self):
        super().__init__('position_controller')

        # --- Parameters (modify these or pass via ROS 2 param file) ---
        self.declare_parameter('takeoff_height', 2.5)          # [m] (positive = up)
        self.declare_parameter('hover_x', 0.0)                 # [m] NED
        self.declare_parameter('hover_y', 0.0)                 # [m] NED
        self.declare_parameter('hover_z', -2.5)                # [m] NED (negative = up)
        self.declare_parameter('update_rate', 50.0)            # [Hz] setpoint stream rate

        self._takeoff_height = self.get_parameter('takeoff_height').value
        self._hover_pos = [
            self.get_parameter('hover_x').value,
            self.get_parameter('hover_y').value,
            self.get_parameter('hover_z').value,
        ]
        self._update_period = 1.0 / self.get_parameter('update_rate').value

        # --- QoS: best-effort for sensor-like streams ---
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Publishers ---
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_best_effort)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_best_effort)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_best_effort)

        # --- Timer: heartbeat for offboard mode + setpoint stream ---
        self._timer = self.create_timer(self._update_period, self._timer_callback)

        # --- State machine ---
        self._state = 'INIT'
        self._state_start_time = self.get_clock().now()
        self._offboard_setpoint_counter = 0
        self._last_setpoint = TrajectorySetpoint()
        self._last_setpoint.position = [float('nan')] * 3
        self._last_setpoint.yaw = float('nan')

        self.get_logger().info('PositionController started')

    # ------------------------------------------------------------------
    #  State machine
    # ------------------------------------------------------------------

    def _set_state(self, new_state: str):
        self.get_logger().info(f'State: {self._state} -> {new_state}')
        self._state = new_state
        self._state_start_time = self.get_clock().now()

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self._state_start_time).nanoseconds * 1e-9

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = float('nan')):
        """Publish a position setpoint in NED frame (z negative = up)."""
        sp = TrajectorySetpoint()
        sp.position = [x, y, z]
        sp.yaw = yaw
        self._last_setpoint = sp
        self._trajectory_pub.publish(sp)

    def _publish_offboard_mode(self):
        """Tell PX4 to accept position setpoints."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_mode_pub.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        """Send a VehicleCommand."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(msg)

    def _arm(self):
        self.get_logger().info('Sending ARM command')
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _disarm(self):
        self.get_logger().info('Sending DISARM command')
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def _set_offboard_mode(self):
        """Switch to OFFBOARD mode.
           param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
           param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD (6)
        """
        self.get_logger().info('Requesting OFFBOARD mode')
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    # ------------------------------------------------------------------
    #  Timer callback — main control loop
    # ------------------------------------------------------------------

    def _timer_callback(self):
        # Always stream offboard mode + setpoints so PX4 sees
        # the offboard stream before we request the mode switch.
        self._publish_offboard_mode()

        if self._state == 'INIT':
            # Stream setpoints at origin first — PX4 needs to see >2 Hz
            # of OffboardControlMode before accepting the mode switch.
            self._publish_setpoint(0.0, 0.0, 0.0, 0.0)
            if self._elapsed_s() > 2.0:
                self._set_state('ARMING')

        elif self._state == 'ARMING':
            self._publish_setpoint(0.0, 0.0, 0.0, 0.0)
            # Send arm ONCE at start of this state
            if self._elapsed_s() < 0.1:
                self._arm()
            if self._elapsed_s() > 2.0:
                self._set_state('SET_OFFBOARD')

        elif self._state == 'SET_OFFBOARD':
            self._publish_setpoint(0.0, 0.0, 0.0, 0.0)
            # Request offboard mode ONCE
            if self._elapsed_s() < 0.1:
                self._set_offboard_mode()
            if self._elapsed_s() > 2.0:
                self._set_state('TAKEOFF')

        elif self._state == 'TAKEOFF':
            self._publish_setpoint(0.0, 0.0, -self._takeoff_height, 0.0)
            # Re-arm in case we lost it
            if self._elapsed_s() < 0.1:
                self._arm()
            if self._elapsed_s() > 5.0:
                self._set_state('HOVER')

        elif self._state == 'HOVER':
            self._publish_setpoint(
                self._hover_pos[0],
                self._hover_pos[1],
                self._hover_pos[2],
            )

    # ------------------------------------------------------------------
    #  Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._disarm()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
