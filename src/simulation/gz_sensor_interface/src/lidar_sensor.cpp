// lidar_sensor — express the remaining level Gazebo LiDAR in base_link.
//
// The raw gpu_lidar cloud is sensor-local. This node applies the fixed
// horizontal LiDAR mounting transform and republishes it for FAST-LIO/SUPER.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "gz_sensor_interface/stamp_util.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using geometry_msgs::msg::TransformStamped;

namespace gz_sensor_interface
{
namespace
{
struct Extrinsic
{
  float t[3];
  float roll;
};

void transformPoint(const Extrinsic &ext, float &x, float &y, float &z)
{
  const float c = std::cos(ext.roll);
  const float s = std::sin(ext.roll);
  const float yr = y * c - z * s;
  const float zr = y * s + z * c;
  x += ext.t[0];
  y = yr + ext.t[1];
  z = zr + ext.t[2];
}
}  // namespace

class LidarSensor : public rclcpp::Node
{
public:
  LidarSensor() : Node("lidar_sensor")
  {
    declare_parameter("input_horizontal", "/swan_gamma_v2/scan_horizontal/points");
    declare_parameter("output_horizontal", "/swan_gamma_v2/scan_horizontal/points_body");
    declare_parameter("horizontal.t", std::vector<double>{0.0, 0.0, 0.16});
    declare_parameter("horizontal.roll", 0.0);

    const auto translation = get_parameter("horizontal.t").as_double_array();
    for (std::size_t index = 0; index < translation.size() && index < 3; ++index) {
      horizontal_ext_.t[index] = static_cast<float>(translation[index]);
    }
    horizontal_ext_.roll = static_cast<float>(get_parameter("horizontal.roll").as_double());

    const std::string input = get_parameter("input_horizontal").as_string();
    const std::string output = get_parameter("output_horizontal").as_string();
    const auto qos = rclcpp::QoS(5).best_effort();
    sub_ = create_subscription<PointCloud2>(
        input, qos, [this](const PointCloud2::SharedPtr message) { onCloud(message); });
    pub_ = create_publisher<PointCloud2>(output, qos);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTf();
    RCLCPP_INFO(get_logger(), "lidar_sensor: %s -> %s (base_link)",
                input.c_str(), output.c_str());
  }

private:
  void publishStaticTf()
  {
    TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "lidar_link";
    transform.transform.translation.x = horizontal_ext_.t[0];
    transform.transform.translation.y = horizontal_ext_.t[1];
    transform.transform.translation.z = horizontal_ext_.t[2];
    transform.transform.rotation.w = std::cos(horizontal_ext_.roll / 2.0F);
    transform.transform.rotation.x = std::sin(horizontal_ext_.roll / 2.0F);
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(transform);
  }

  void onCloud(const PointCloud2::SharedPtr message)
  {
    const auto payload = transformCloud(message);
    if (!payload || payload->second == 0) {
      return;
    }
    pub_->publish(buildBodyCloud(payload->first, payload->second, message->header.stamp));
  }

  PointCloud2 buildBodyCloud(const std::vector<std::uint8_t> &data, std::uint32_t count,
                             const builtin_interfaces::msg::Time &stamp)
  {
    PointCloud2 output;
    output.header.frame_id = "base_link";
    output.header.stamp = monotonic_stamp_.clamp(stamp);
    output.height = 1;
    output.width = count;
    output.fields = {
      PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
    };
    output.is_bigendian = false;
    output.point_step = 16;
    output.row_step = output.point_step * count;
    output.data = data;
    output.is_dense = true;
    return output;
  }

  std::optional<std::pair<std::vector<std::uint8_t>, std::uint32_t>>
  transformCloud(const PointCloud2::SharedPtr &message) const
  {
    const std::uint32_t count = message->width * message->height;
    const std::uint32_t step = message->point_step;
    std::int32_t offsets[4] = {-1, -1, -1, -1};
    const char *names[4] = {"x", "y", "z", "intensity"};
    for (const PointField &field : message->fields) {
      for (int index = 0; index < 4; ++index) {
        if (field.name == names[index]) {
          offsets[index] = static_cast<std::int32_t>(field.offset);
        }
      }
    }
    if (step == 0 || count == 0 || offsets[0] < 0 || offsets[1] < 0 || offsets[2] < 0 ||
        offsets[0] + 4 > static_cast<std::int32_t>(step) ||
        offsets[1] + 4 > static_cast<std::int32_t>(step) ||
        offsets[2] + 4 > static_cast<std::int32_t>(step)) {
      return std::nullopt;
    }
    if (offsets[3] + 4 > static_cast<std::int32_t>(step)) {
      offsets[3] = -1;
    }

    constexpr std::uint32_t output_step = 16;
    std::vector<std::uint8_t> data;
    data.reserve(static_cast<std::size_t>(count) * output_step);
    for (std::uint32_t index = 0; index < count; ++index) {
      const auto *source = message->data.data() + static_cast<std::size_t>(index) * step;
      float x;
      float y;
      float z;
      std::memcpy(&x, source + offsets[0], sizeof(float));
      std::memcpy(&y, source + offsets[1], sizeof(float));
      std::memcpy(&z, source + offsets[2], sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      transformPoint(horizontal_ext_, x, y, z);
      float intensity = 0.0F;
      if (offsets[3] >= 0) {
        std::memcpy(&intensity, source + offsets[3], sizeof(float));
      }
      const std::size_t destination = data.size();
      data.resize(destination + output_step);
      std::memcpy(data.data() + destination, &x, sizeof(float));
      std::memcpy(data.data() + destination + 4, &y, sizeof(float));
      std::memcpy(data.data() + destination + 8, &z, sizeof(float));
      std::memcpy(data.data() + destination + 12, &intensity, sizeof(float));
    }
    const auto kept = static_cast<std::uint32_t>(data.size() / output_step);
    return std::make_pair(std::move(data), kept);
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  Extrinsic horizontal_ext_{};
  StampMonotonicizer monotonic_stamp_;
};
}  // namespace gz_sensor_interface

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_interface::LidarSensor>());
  rclcpp::shutdown();
  return 0;
}
