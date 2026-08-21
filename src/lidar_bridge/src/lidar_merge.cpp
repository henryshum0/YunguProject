// lidar_merge — Fuse two side-mounted LiDAR clouds into one base_link scan.
//
// Mirrors the real hardware: one LiDAR on each side of the drone (tilted down
// ~35°). Each cloud is transformed into base_link using its mounting
// extrinsics, then concatenated into a single PointCloud2 for FAST-LIO.
// Non-finite (NaN) points are dropped. The fused stamp is monotonicized (see
// stamp_util.hpp) so FAST-LIO never sees a regressing scan time.
//
// Extrinsics (lidar frame -> base_link), matching model.sdf (swan_gamma_v2):
//   left  lidar: pose (0, +0.40, 0.05, roll -0.6)
//   right lidar: pose (0, -0.40, 0.05, roll +0.6)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "lidar_bridge/stamp_util.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace lidar_bridge
{

namespace
{
struct Extrinsic
{
  float t[3];
  float roll;
};

// roll-only rotation about the forward (x) axis; (px,py,pz) -> (px, y' , z').
struct Rotation
{
  float c;
  float s;
};

Rotation makeRotation(float roll)
{
  return {std::cos(roll), std::sin(roll)};
}

// Rotate (y,z) by roll about x, then add translation.
void transformPoint(const Rotation &rot, const Extrinsic &ext,
                    float &x, float &y, float &z)
{
  const float yr = y * rot.c - z * rot.s;
  const float zr = y * rot.s + z * rot.c;
  x += ext.t[0];
  y = yr + ext.t[1];
  z = zr + ext.t[2];
}

}  // namespace

class LidarMerge : public rclcpp::Node
{
public:
  LidarMerge() : Node("lidar_merge")
  {
    declare_parameter("input_left", "/swan_gamma_v2/scan_left/points");
    declare_parameter("input_right", "/swan_gamma_v2/scan_right/points");
    declare_parameter("output_topic", "/swan_gamma_v2/scan/points_fused");

    const std::string in_left = get_parameter("input_left").as_string();
    const std::string in_right = get_parameter("input_right").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();

    auto qos = rclcpp::QoS(5).best_effort();
    sub_left_ = create_subscription<PointCloud2>(
        in_left, qos, [this](const PointCloud2::SharedPtr m) { onCloud("left", m); });
    sub_right_ = create_subscription<PointCloud2>(
        in_right, qos, [this](const PointCloud2::SharedPtr m) { onCloud("right", m); });
    pub_ = create_publisher<PointCloud2>(out_topic, qos);

    // Mounting extrinsics (lidar frame -> base_link).
    left_ext_ = {{0.0f, 0.40f, 0.05f}, -0.6f};
    right_ext_ = {{0.0f, -0.40f, 0.05f}, +0.6f};

    RCLCPP_INFO(get_logger(), "lidar_merge: %s + %s -> %s (base_link)",
                in_left.c_str(), in_right.c_str(), out_topic.c_str());
  }

private:
  struct CloudEntry
  {
    std::vector<std::uint8_t> bytes;
    std::uint32_t kept;
    builtin_interfaces::msg::Time stamp;
  };

  void onCloud(const std::string &side, const PointCloud2::SharedPtr msg)
  {
    const auto payload = transformCloud(msg, side);
    if (!payload) {
      return;
    }

    const auto &[bytes, kept] = *payload;
    if (kept == 0) {
      return;
    }
    latest_[side] = CloudEntry{bytes, kept, msg->header.stamp};

    // Merge both sides' latest (reuse the other side if present).
    std::vector<std::uint8_t> out_data;
    std::uint32_t total = 0;
    builtin_interfaces::msg::Time stamp;
    bool have_stamp = false;
    for (const std::string &s : {"left", "right"}) {
      auto it = latest_.find(s);
      if (it == latest_.end()) {
        continue;
      }
      const CloudEntry &e = it->second;
      out_data.insert(out_data.end(), e.bytes.begin(), e.bytes.end());
      total += e.kept;
      if (!have_stamp || stampKey(e.stamp) > stampKey(stamp)) {
        stamp = e.stamp;
        have_stamp = true;
      }
    }
    if (out_data.empty() || total == 0) {
      return;
    }

    auto out = std::make_shared<PointCloud2>();
    out->header.frame_id = "base_link";
    out->header.stamp = mono_.clamp(stamp);
    out->height = 1;
    out->width = total;
    out->fields = msg->fields;
    out->is_bigendian = false;
    out->point_step = msg->point_step;
    out->row_step = msg->point_step * total;
    out->data = std::move(out_data);
    out->is_dense = true;

    ++cb_count_;
    if (mono_.clamped() > 0 && cb_count_ % 500 == 0) {
      RCLCPP_WARN(get_logger(), "lidar_merge: %lu/%lu output stamps clamped "
                                "(non-monotonic sim clock)",
                  static_cast<unsigned long>(mono_.clamped()),
                  static_cast<unsigned long>(cb_count_));
    }
    pub_->publish(*out);
  }

  // Transform x/y/z of every point into base_link, dropping non-finite points.
  std::optional<std::pair<std::vector<std::uint8_t>, std::uint32_t>>
  transformCloud(const PointCloud2::SharedPtr msg, const std::string &side)
  {
    const Extrinsic &ext = (side == "left") ? left_ext_ : right_ext_;
    const Rotation rot = makeRotation(ext.roll);

    const std::uint32_t step = msg->point_step;
    const std::uint32_t n = msg->width * msg->height;
    std::int32_t offs[3] = {-1, -1, -1};
    const char *names[3] = {"x", "y", "z"};
    for (const PointField &f : msg->fields) {
      for (int i = 0; i < 3; ++i) {
        if (f.name == names[i]) {
          offs[i] = static_cast<std::int32_t>(f.offset);
        }
      }
    }
    if (step == 0 || n == 0 || offs[0] < 0 || offs[1] < 0 || offs[2] < 0) {
      return std::nullopt;
    }

    // Transform all points, dropping those that become non-finite.
    std::vector<std::uint8_t> kept_bytes;
    kept_bytes.reserve(static_cast<std::size_t>(n) * step);
    const std::uint8_t *src = msg->data.data();
    for (std::uint32_t i = 0; i < n; ++i) {
      const std::uint8_t *p = src + static_cast<std::size_t>(i) * step;
      float x, y, z;
      std::memcpy(&x, p + offs[0], 4);
      std::memcpy(&y, p + offs[1], 4);
      std::memcpy(&z, p + offs[2], 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      transformPoint(rot, ext, x, y, z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      // Copy the whole point, then overwrite x/y/z with the transformed values.
      const std::size_t dst = kept_bytes.size();
      kept_bytes.resize(dst + step);
      std::memcpy(kept_bytes.data() + dst, p, step);
      std::memcpy(kept_bytes.data() + dst + offs[0], &x, 4);
      std::memcpy(kept_bytes.data() + dst + offs[1], &y, 4);
      std::memcpy(kept_bytes.data() + dst + offs[2], &z, 4);
    }

    return std::make_pair(std::move(kept_bytes),
                          static_cast<std::uint32_t>(kept_bytes.size() / step));
  }

  static std::uint64_t stampKey(const builtin_interfaces::msg::Time &t)
  {
    return static_cast<std::uint64_t>(t.sec) * 1000000000ULL + t.nanosec;
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_left_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_right_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;

  Extrinsic left_ext_;
  Extrinsic right_ext_;
  std::unordered_map<std::string, CloudEntry> latest_;
  StampMonotonicizer mono_;
  std::uint64_t cb_count_{0};
};

}  // namespace lidar_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_bridge::LidarMerge>());
  rclcpp::shutdown();
  return 0;
}
