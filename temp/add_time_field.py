#!/usr/bin/env python3
"""
add_time_field.py — Add a 'time' field to PointCloud2 for FAST-LIO compatibility.

Gazebo's gpu_lidar outputs PointCloud2 with fields (x, y, z, intensity) but
FAST-LIO requires a 'time' field for each point (used in deskewing/motion
compensation). This relay node adds a zero-filled 'time' field.

Usage:
  source install/setup.bash
  python3 temp/add_time_field.py
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField


class AddTimeField(Node):
    def __init__(self):
        super().__init__("add_time_field",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter("input_topic", "/x500_lidar/scan/points")
        self.declare_parameter("output_topic", "/x500_lidar/scan/points_timed")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._cb, rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self._pub = self.create_publisher(
            PointCloud2, out_topic, rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info(f"Adding time field: {in_topic} → {out_topic}")

    def _cb(self, cloud: PointCloud2):
        # Check if 'time' field already exists
        field_names = [f.name for f in cloud.fields]
        if 'time' in field_names:
            self._pub.publish(cloud)
            return

        # Compute point count and data layout
        point_step_old = cloud.point_step
        n_points = cloud.width * cloud.height

        # New layout: original fields + time (float32, 4 bytes)
        time_offset = point_step_old
        new_point_step = point_step_old + 4  # add float32 time
        new_row_step = new_point_step * cloud.width
        new_data = bytearray(n_points * new_point_step)

        import struct
        # timestamp_unit=2 (microseconds), scan_period=1/10Hz=0.1s=100,000us
        t_step = 100000.0 / n_points if n_points > 0 else 0.0

        for i in range(n_points):
            src_start = i * point_step_old
            dst_start = i * new_point_step
            new_data[dst_start:dst_start + point_step_old] = \
                cloud.data[src_start:src_start + point_step_old]
            # time field: float32 = point timestamp in microseconds
            t_us = i * t_step
            new_data[dst_start + time_offset:dst_start + new_point_step] = \
                struct.pack('<f', t_us)

        # Build new message
        new_cloud = PointCloud2()
        new_cloud.header = cloud.header
        new_cloud.height = cloud.height
        new_cloud.width = cloud.width
        new_cloud.is_dense = cloud.is_dense
        new_cloud.is_bigendian = cloud.is_bigendian
        new_cloud.point_step = new_point_step
        new_cloud.row_step = new_row_step
        new_cloud.data = bytes(new_data)

        # Copy original fields + add 'time'
        new_cloud.fields = list(cloud.fields)
        time_field = PointField()
        time_field.name = 'time'
        time_field.offset = time_offset
        time_field.datatype = PointField.FLOAT32
        time_field.count = 1
        new_cloud.fields.append(time_field)

        self._pub.publish(new_cloud)


def main():
    rclpy.init()
    rclpy.spin(AddTimeField())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
