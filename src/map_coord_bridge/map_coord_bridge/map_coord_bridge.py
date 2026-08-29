#!/usr/bin/env python3
"""map_coord_bridge — bidirectional ENU bridge between the coverage planner's
local frame (map_id e.g. yungu2030_local_origin) and the sim world frame.

Two independent channels:

  map -> world : subscribe  map_in_topic    (PoseStamped, planner ENU)
                 publish    world_out_topic (PoseStamped, world ENU)
                 Intended as the /waypoint_pose feed for manual or external
                 map-frame goals (QoS is best_effort to match goal_marker).

  world -> map : subscribe  world_in_topic  (Odometry, world ENU, e.g.
                 /lidar_slam/odom)
                 publish    map_out_topic   (Odometry, planner ENU)
                 Intended for showing the vehicle on the planner map
                 (verification of calibration T, coverage overlays).

The transform is p_world = Rz(yaw_deg) * p_map + T (see transform.py).
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from map_coord_bridge.transform import (
    to_world, to_map, yaw_from_quat, quat_from_yaw,
)

# best_effort: can subscribe to any publisher, and matches goal_marker_node's
# best_effort subscription on /waypoint_pose.
QOS_BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
QOS_REL = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class MapCoordBridge(Node):
    def __init__(self, offset=(0.0, 0.0, 0.0), yaw_deg=0.0,
                 map_in_topic="/map_pose_in",
                 world_out_topic="/waypoint_pose",
                 world_in_topic="/lidar_slam/odom",
                 map_out_topic="/map/vehicle_odom"):
        super().__init__("map_coord_bridge")
        self._offset = offset
        self._yaw_deg = yaw_deg

        # map -> world (PoseStamped)
        self._map_in = self.create_subscription(
            PoseStamped, map_in_topic, self._on_map_pose, QOS_BE)
        self._world_out = self.create_publisher(
            PoseStamped, world_out_topic, QOS_BE)

        # world -> map (Odometry)
        self._world_in = self.create_subscription(
            Odometry, world_in_topic, self._on_world_odom, QOS_BE)
        self._map_out = self.create_publisher(
            Odometry, map_out_topic, QOS_REL)

        self.get_logger().info(
            f"bridge: {map_in_topic} -> {world_out_topic}  |  "
            f"{world_in_topic} -> {map_out_topic}")
        self.get_logger().info(
            f"transform: Rz({yaw_deg} deg) + T=({offset[0]}, {offset[1]}, {offset[2]})")

    # -- map -> world ------------------------------------------------------

    def _on_map_pose(self, msg: PoseStamped):
        p = msg.pose.position
        x, y, z = to_world(p.x, p.y, p.z, self._yaw_deg, *self._offset)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.position.z = z
        # Rotate the map-frame yaw by the bridge yaw (z-axis compound = add).
        yaw = yaw_from_quat(msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w)
        qx, qy, qz, qw = quat_from_yaw(yaw + math.radians(self._yaw_deg))
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._world_out.publish(out)
        self.get_logger().info(
            f"map pose ({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) -> "
            f"world ({x:.1f}, {y:.1f}, {z:.1f})")

    # -- world -> map ------------------------------------------------------

    def _on_world_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        x, y, z = to_map(p.x, p.y, p.z, self._yaw_deg, *self._offset)
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        # Back-rotate the world yaw into the map frame (subtract the bridge yaw).
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        qx, qy, qz, qw = quat_from_yaw(yaw - math.radians(self._yaw_deg))
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        self._map_out.publish(out)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Bidirectional ENU bridge: planner-local frame <-> world frame.")
    parser.add_argument("--offset-x", type=float, default=0.0, help="T_x [m]")
    parser.add_argument("--offset-y", type=float, default=0.0, help="T_y [m]")
    parser.add_argument("--offset-z", type=float, default=0.0, help="T_z [m]")
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="map->world rotation about z [deg]")
    parser.add_argument("--map-in-topic", default="/map_pose_in",
                        help="PoseStamped in planner ENU (default %(default)s)")
    parser.add_argument("--world-out-topic", default="/waypoint_pose",
                        help="PoseStamped out in world ENU (default %(default)s)")
    parser.add_argument("--world-in-topic", default="/lidar_slam/odom",
                        help="Odometry in world ENU (default %(default)s)")
    parser.add_argument("--map-out-topic", default="/map/vehicle_odom",
                        help="Odometry out in planner ENU (default %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = MapCoordBridge(
        offset=(args.offset_x, args.offset_y, args.offset_z),
        yaw_deg=args.yaw_deg,
        map_in_topic=args.map_in_topic,
        world_out_topic=args.world_out_topic,
        world_in_topic=args.world_in_topic,
        map_out_topic=args.map_out_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
