// add_time_field — Add a 'time' field to PointCloud2 for FAST-LIO and
// monotonicize the scan header stamp.
//
// Gazebo's gpu_lidar outputs PointCloud2 with fields (x, y, z, intensity) but
// FAST-LIO requires a 'time' field per point (used in deskewing). This node
// appends a zero-filled float32 'time' field. It also clamps the forwarded
// header stamp to a strictly increasing sequence (see stamp_util.hpp) so
// FAST-LIO never sees a regressing scan time.
//
// Usage:
//   ros2 run lidar_bridge add_time_field --ros-args \
//     -p input_topic:=/swan_gamma_v2/scan_left/points \
//     -p output_topic:=/swan_gamma_v2/scan_left/points_timed
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "lidar_bridge/stamp_util.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace lidar_bridge
{

class AddTimeField : public rclcpp::Node
{
public:
  AddTimeField() : Node("add_time_field")
  {
    declare_parameter("input_topic", "/x500_lidar/scan/points");
    declare_parameter("output_topic", "/x500_lidar/scan/points_timed");

    const std::string in_topic = get_parameter("input_topic").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();

    auto qos = rclcpp::QoS(5).best_effort();
    sub_ = create_subscription<PointCloud2>(
        in_topic, qos,
        [this](const PointCloud2::SharedPtr cloud) { onCloud(cloud); });
    pub_ = create_publisher<PointCloud2>(out_topic, qos);

    RCLCPP_INFO(get_logger(), "add_time_field: %s -> %s", in_topic.c_str(),
                out_topic.c_str());
  }

private:
  void onCloud(const PointCloud2::SharedPtr cloud)
  {
    ++cb_count_;
    auto out = std::make_shared<PointCloud2>();
    *out = *cloud;
    out->header.stamp = mono_.clamp(out->header.stamp);

    const bool has_time = std::any_of(
        cloud->fields.begin(), cloud->fields.end(),
        [](const PointField &f) { return f.name == "time"; });

    if (!has_time) {
      // Append a zero-filled float32 'time' field at the end of each point.
      const std::uint32_t point_step = cloud->point_step;
      const std::uint32_t n = cloud->width * cloud->height;
      out->point_step = point_step + 4;
      out->row_step = out->point_step * cloud->width;
      out->data.resize(static_cast<std::size_t>(n) * out->point_step);

      // Copy existing bytes, then zero the tail (the new time field).
      for (std::uint32_t i = 0; i < n; ++i) {
        const std::size_t src = static_cast<std::size_t>(i) * point_step;
        const std::size_t dst = static_cast<std::size_t>(i) * out->point_step;
        std::memcpy(out->data.data() + dst, cloud->data.data() + src, point_step);
        std::memset(out->data.data() + dst + point_step, 0, 4);
      }

      PointField time_field;
      time_field.name = "time";
      time_field.offset = point_step;
      time_field.datatype = PointField::FLOAT32;
      time_field.count = 1;
      out->fields.push_back(time_field);
    }

    maybeLogWarning();
    pub_->publish(*out);
  }

  void maybeLogWarning()
  {
    if (mono_.clamped() > 0 && cb_count_ % 500 == 0) {
      RCLCPP_WARN(get_logger(), "add_time_field: %lu/%lu stamps clamped "
                                "(non-monotonic sim clock)",
                  static_cast<unsigned long>(mono_.clamped()),
                  static_cast<unsigned long>(cb_count_));
    }
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  StampMonotonicizer mono_;
  std::uint64_t cb_count_{0};
};

}  // namespace lidar_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_bridge::AddTimeField>());
  rclcpp::shutdown();
  return 0;
}
