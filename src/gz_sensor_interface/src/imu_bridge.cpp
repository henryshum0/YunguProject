// imu_bridge — relay the Gazebo IMU (/livox/imu_raw) onto /livox/imu with
// monotonicized timestamps.
//
// FAST-LIO consumes /livox/imu; the raw bridge output can carry regressing /
// duplicated sim-time stamps, which make the EKF abort. This node clamps every
// stamp to a strictly increasing sequence (see stamp_util.hpp).
//
// Frame note: the Gazebo imu_sensor is mounted directly in base_link with no
// pose offset/rotation, so its angular_velocity / linear_acceleration are
// already expressed in the body frame (base_link, FLU) — exactly the frame
// FAST-LIO integrates in. FAST-LIO does NOT consume a world-frame IMU; it
// estimates the world pose itself from body-frame IMU + LiDAR. We therefore do
// NOT rotate the IMU here. We only stamp the frame_id (base_link) explicitly,
// since the raw bridge output leaves it unset/ambiguous.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>
#include <string>

#include "gz_sensor_interface/stamp_util.hpp"

using sensor_msgs::msg::Imu;

namespace gz_sensor_interface
{

class ImuBridge : public rclcpp::Node
{
public:
  ImuBridge() : Node("imu_bridge")
  {
    declare_parameter("input_topic", "/livox/imu_raw");
    declare_parameter("output_topic", "/livox/imu");
    // Body frame the IMU is expressed in (Gazebo imu_sensor is in base_link).
    declare_parameter("frame_id", "base_link");
    const std::string in = get_parameter("input_topic").as_string();
    const std::string out = get_parameter("output_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    auto qos_sub = rclcpp::QoS(20).best_effort();
    auto qos_pub = rclcpp::QoS(10).reliable();
    sub_ = create_subscription<Imu>(
        in, qos_sub, [this](const Imu::SharedPtr m) { onImu(m); });
    pub_ = create_publisher<Imu>(out, qos_pub);
    RCLCPP_INFO(get_logger(), "imu_bridge: %s -> %s (monotonic stamps)", in.c_str(), out.c_str());
  }

private:
  void onImu(const Imu::SharedPtr m)
  {
    auto out = std::make_shared<Imu>(*m);
    out->header.stamp = mono_.clamp(m->header.stamp);
    out->header.frame_id = frame_id_;
    ++cb_count_;
    if (mono_.clamped() > 0 && cb_count_ % 500 == 0) {
      RCLCPP_WARN(get_logger(), "imu_bridge: %lu/%lu stamps clamped (non-monotonic sim clock)",
                  static_cast<unsigned long>(mono_.clamped()),
                  static_cast<unsigned long>(cb_count_));
    }
    pub_->publish(*out);
  }

  rclcpp::Subscription<Imu>::SharedPtr sub_;
  rclcpp::Publisher<Imu>::SharedPtr pub_;
  std::string frame_id_;
  StampMonotonicizer mono_;
  std::uint64_t cb_count_{0};
};

}  // namespace gz_sensor_interface

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_interface::ImuBridge>());
  rclcpp::shutdown();
  return 0;
}
