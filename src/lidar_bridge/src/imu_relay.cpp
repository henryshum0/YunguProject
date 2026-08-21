// imu_relay — Monotonicize gz IMU timestamps before FAST-LIO.
//
// gz IMU is bridged to /livox/imu_raw with the sim clock (same source as the
// lidar clouds). FAST-LIO aborts on regressing IMU stamps ("cannot store a
// negative time point") and at 250 Hz DDS delivery order can jitter, so a
// non-monotonic stamp made FAST-LIO diverge. This relay clamps stamps to a
// strictly increasing sequence (see stamp_util.hpp).
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>

#include "lidar_bridge/stamp_util.hpp"

using sensor_msgs::msg::Imu;

namespace lidar_bridge
{

class ImuRelay : public rclcpp::Node
{
public:
  ImuRelay() : Node("imu_relay")
  {
    constexpr const char *kIn = "/livox/imu_raw";
    constexpr const char *kOut = "/livox/imu";

    // Gazebo bridged IMU arrives best_effort.
    sub_ = create_subscription<Imu>(
        kIn, rclcpp::QoS(20).best_effort(),
        [this](const Imu::SharedPtr msg) { onImu(msg); });
    // FAST-LIO subscribes with default (reliable) QoS — reliable output.
    pub_ = create_publisher<Imu>(kOut, rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "imu_relay: %s -> %s (monotonic)", kIn, kOut);
  }

private:
  void onImu(const Imu::SharedPtr msg)
  {
    ++cb_count_;
    msg->header.stamp = mono_.clamp(msg->header.stamp);

    if (mono_.clamped() > 0 && cb_count_ % 500 == 0) {
      RCLCPP_WARN(get_logger(), "imu_relay: %lu/%lu stamps clamped "
                                "(non-monotonic gz IMU)",
                  static_cast<unsigned long>(mono_.clamped()),
                  static_cast<unsigned long>(cb_count_));
    }
    pub_->publish(*msg);
  }

  rclcpp::Subscription<Imu>::SharedPtr sub_;
  rclcpp::Publisher<Imu>::SharedPtr pub_;
  StampMonotonicizer mono_;
  std::uint64_t cb_count_{0};
};

}  // namespace lidar_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_bridge::ImuRelay>());
  rclcpp::shutdown();
  return 0;
}
