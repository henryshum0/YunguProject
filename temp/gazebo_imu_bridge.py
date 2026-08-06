#!/usr/bin/env python3
"""PX4 SensorCombined → sensor_msgs/Imu. Timestamps from ROS clock."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2


class GazeboImuBridge(Node):
    def __init__(self):
        super().__init__("gazebo_imu_bridge")

        self.declare_parameter("input_topic", "/fmu/out/sensor_combined")
        self.declare_parameter("output_topic", "/livox/imu")
        self.declare_parameter("frame_id", "body")
        self.declare_parameter("lidar_topic", "/x500_lidar/scan/points")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        lidar_topic = self.get_parameter("lidar_topic").value
        self._frame_id = self.get_parameter("frame_id").value

        qos_be = QoSProfile(depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._sub = self.create_subscription(SensorCombined, in_topic, self._cb, qos_be)
        self._pub = self.create_publisher(Imu, out_topic, QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))

        # Sniff LiDAR topic to get sim time reference
        self._lidar_t0 = 0.0   # first LiDAR timestamp (sim seconds)
        self._px4_t0 = 0
        self._got_lidar = False
        self._got_px4 = False
        self._offset_done = False
        self._lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._lidar_cb, qos_be)

        self._count = 0
        self.get_logger().info(f"IMU bridge: {in_topic} → {out_topic}")

    def _lidar_cb(self, msg: PointCloud2):
        if not self._got_lidar:
            self._lidar_t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._got_lidar = True

    def _cb(self, msg: SensorCombined):
        px4_us = msg.timestamp
        if not self._got_px4:
            self._px4_t0 = px4_us
            self._got_px4 = True
        if not (self._got_lidar and self._got_px4):
            return
        if not self._offset_done:
            self._offset_done = True

        # IMU time = first LiDAR time + elapsed PX4 time
        dt = (px4_us - self._px4_t0) * 1e-6
        sim_t = self._lidar_t0 + dt

        imu = Imu()
        # Guard against negative timestamps (sim_t < 0 when lidar ref is in future)
        sec_part = int(sim_t)
        nsec_part = int((sim_t - sec_part) * 1e9)
        if nsec_part < 0:
            sec_part -= 1
            nsec_part += 1000000000
        imu.header.stamp.sec = sec_part
        imu.header.stamp.nanosec = nsec_part
        imu.header.frame_id = self._frame_id
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = float(msg.gyro_rad[0])
        imu.angular_velocity.y = float(msg.gyro_rad[1])
        imu.angular_velocity.z = float(msg.gyro_rad[2])
        imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
        self._pub.publish(imu)

        self._count += 1
        if self._count % 500 == 0:
            self.get_logger().info(f"IMU bridge: {self._count} msgs, t={sim_t:.1f}s")


def main():
    rclpy.init()
    rclpy.spin(GazeboImuBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
