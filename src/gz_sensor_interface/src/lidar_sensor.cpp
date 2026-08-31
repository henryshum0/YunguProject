// lidar_sensor — transform left/right/horizontal LiDARs into base_link and
// publish four body-frame outputs (merged + left + right + top).
//
// Gazebo's gpu_lidar emits points in the sensor-local frame (the lidar_left_link
// / lidar_right_link / lidar_h_link bodies). This node applies each sensor's
// mounting extrinsics to express the points in the vehicle body frame
// (base_link), then publishes:
//   merged      : the two side LiDARs fused into one base_link scan
//   left        : the left side LiDAR alone, in base_link
//   right       : the right side LiDAR alone, in base_link
//   top         : the horizontal (level) LiDAR alone, in base_link
//
// Mounting extrinsics (lidar frame -> base_link), matching model.sdf
// (swan_gamma_v2):
//   left       lidar: pose (0, +0.40, 0.05, roll -0.6)
//   right      lidar: pose (0, -0.40, 0.05, roll +0.6)
//   horizontal lidar: pose (0, 0, 0.16, roll 0)   (level LiDAR, +z only)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>
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

class LidarSensor : public rclcpp::Node
{
public:
  LidarSensor() : Node("lidar_sensor")
  {
    // NOTE: output topics are deliberately DIFFERENT from the inputs (the raw
    // Gazebo /scan_* topics). Keeping them equal would make the node subscribe
    // to its own output and re-apply the transform on every cycle.
    declare_parameter("input_left", "/swan_gamma_v2/scan_left/points");
    declare_parameter("input_right", "/swan_gamma_v2/scan_right/points");
    declare_parameter("input_horizontal", "/swan_gamma_v2/scan_horizontal/points");
    declare_parameter("output_merged", "/swan_gamma_v2/scan/points_fused");
    declare_parameter("output_left", "/swan_gamma_v2/scan_left/points_body");
    declare_parameter("output_right", "/swan_gamma_v2/scan_right/points_body");
    declare_parameter("output_top", "/swan_gamma_v2/scan_horizontal/points_body");
    // Max allowed gap [s] between the left/right scans being fused.
    declare_parameter("time_sync_tol", 0.05);
    // Mounting extrinsics (lidar frame -> base_link), matching model.sdf.
    declare_parameter("left.t", std::vector<double>{0.0, 0.40, 0.05});
    declare_parameter("left.roll", -0.6);
    declare_parameter("right.t", std::vector<double>{0.0, -0.40, 0.05});
    declare_parameter("right.roll", 0.6);
    declare_parameter("horizontal.t", std::vector<double>{0.0, 0.0, 0.16});
    declare_parameter("horizontal.roll", 0.0);

    const std::string in_left = get_parameter("input_left").as_string();
    const std::string in_right = get_parameter("input_right").as_string();
    const std::string in_h = get_parameter("input_horizontal").as_string();
    const std::string out_merged = get_parameter("output_merged").as_string();
    const std::string out_left = get_parameter("output_left").as_string();
    const std::string out_right = get_parameter("output_right").as_string();
    const std::string out_top = get_parameter("output_top").as_string();
    const double tol = get_parameter("time_sync_tol").as_double();
    time_sync_tol_ns_ = static_cast<std::uint64_t>(tol * 1e9);

    left_ext_ = makeExtrinsic("left");
    right_ext_ = makeExtrinsic("right");
    top_ext_ = makeExtrinsic("horizontal");

    auto qos = rclcpp::QoS(5).best_effort();
    sub_left_ = create_subscription<PointCloud2>(
        in_left, qos,
        [this](const PointCloud2::SharedPtr m) { onSide("left", m); });
    sub_right_ = create_subscription<PointCloud2>(
        in_right, qos,
        [this](const PointCloud2::SharedPtr m) { onSide("right", m); });
    sub_top_ = create_subscription<PointCloud2>(
        in_h, qos,
        [this](const PointCloud2::SharedPtr m) { onTop(m); });
    pub_merged_ = create_publisher<PointCloud2>(out_merged, qos);
    pub_left_ = create_publisher<PointCloud2>(out_left, qos);
    pub_right_ = create_publisher<PointCloud2>(out_right, qos);
    pub_top_ = create_publisher<PointCloud2>(out_top, qos);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTf("lidar_left_link", left_ext_);
    publishStaticTf("lidar_right_link", right_ext_);
    publishStaticTf("lidar_h_link", top_ext_);

    RCLCPP_INFO(get_logger(),
                "lidar_sensor: {%s, %s, %s} -> {merged=%s, left=%s, right=%s, "
                "top=%s} (all base_link); base_link->sensor TFs published",
                in_left.c_str(), in_right.c_str(), in_h.c_str(),
                out_merged.c_str(), out_left.c_str(), out_right.c_str(),
                out_top.c_str());
  }

private:
  struct CloudEntry
  {
    std::vector<std::uint8_t> bytes;
    std::uint32_t kept;
    builtin_interfaces::msg::Time stamp;
  };

  Extrinsic makeExtrinsic(const std::string &prefix) const
  {
    Extrinsic e{};
    const std::vector<double> t = get_parameter(prefix + ".t").as_double_array();
    for (std::size_t i = 0; i < 3 && i < t.size(); ++i) {
      e.t[i] = static_cast<float>(t[i]);
    }
    e.roll = static_cast<float>(get_parameter(prefix + ".roll").as_double());
    return e;
  }

  void publishStaticTf(const std::string &child, const Extrinsic &ext)
  {
    TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = child;
    tf.transform.translation.x = ext.t[0];
    tf.transform.translation.y = ext.t[1];
    tf.transform.translation.z = ext.t[2];
    // R = Rx(roll) about x: q = (w, x, y, z) = (cos(r/2), sin(r/2), 0, 0).
    tf.transform.rotation.w = std::cos(ext.roll / 2.0);
    tf.transform.rotation.x = std::sin(ext.roll / 2.0);
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(tf);
  }

  // Build a standard 16-byte x/y/z/intensity PointCloud2 in base_link from a
  // transformed payload.
  PointCloud2 buildBodyCloud(const std::vector<std::uint8_t> &data,
                             std::uint32_t n,
                             const builtin_interfaces::msg::Time &stamp)
  {
    PointCloud2 out;
    out.header.frame_id = "base_link";
    out.header.stamp = mono_.clamp(stamp);
    out.height = 1;
    out.width = n;
    out.fields = {
      PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
      PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
    };
    out.is_bigendian = false;
    out.point_step = 16;
    out.row_step = 16 * n;
    out.data = data;
    out.is_dense = true;
    return out;
  }

  void onSide(const std::string &side, const PointCloud2::SharedPtr msg)
  {
    const Extrinsic &ext = (side == "left") ? left_ext_ : right_ext_;
    const auto payload = transformCloud(msg, ext);
    if (!payload) {
      return;
    }
    const auto &[bytes, kept] = *payload;
    if (kept == 0) {
      return;
    }
    latest_[side] = CloudEntry{bytes, kept, msg->header.stamp};

    // Publish this side independently.
    const PointCloud2 solo = buildBodyCloud(bytes, kept, msg->header.stamp);
    if (side == "left") {
      pub_left_->publish(solo);
    } else {
      pub_right_->publish(solo);
    }

    publishMerged(msg->header.stamp);
  }

  void onTop(const PointCloud2::SharedPtr msg)
  {
    const auto payload = transformCloud(msg, top_ext_);
    if (!payload) {
      return;
    }
    const auto &[bytes, kept] = *payload;
    if (kept == 0) {
      return;
    }
    pub_top_->publish(buildBodyCloud(bytes, kept, msg->header.stamp));
  }

  // Merge both sides' latest, but only include a side whose timestamp is within
  // time_sync_tol of the freshest one, so the fused cloud is temporally coherent
  // (needed to align with the simultaneous IMU data). The fused stamp preserves
  // the newest input timestamp.
  void publishMerged(const builtin_interfaces::msg::Time &this_stamp)
  {
    const std::uint64_t this_key = stampKey(this_stamp);
    std::vector<std::uint8_t> out_data;
    std::uint32_t total = 0;
    builtin_interfaces::msg::Time stamp = this_stamp;
    for (const std::string &s : {"left", "right"}) {
      auto it = latest_.find(s);
      if (it == latest_.end()) {
        continue;
      }
      const CloudEntry &e = it->second;
      const std::uint64_t dkey = stampKey(e.stamp);
      if (dkey > this_key && dkey - this_key > time_sync_tol_ns_) {
        continue;  // other side is too far in the future
      }
      if (this_key > dkey && this_key - dkey > time_sync_tol_ns_) {
        continue;  // other side is stale; skip it to keep times close
      }
      out_data.insert(out_data.end(), e.bytes.begin(), e.bytes.end());
      total += e.kept;
      if (dkey > stampKey(stamp)) {
        stamp = e.stamp;
      }
    }
    if (out_data.empty() || total == 0) {
      return;
    }

    PointCloud2 out = buildBodyCloud(out_data, total, stamp);

    ++cb_count_;
    if (mono_.clamped() > 0 && cb_count_ % 500 == 0) {
      RCLCPP_WARN(get_logger(), "lidar_sensor: %lu/%lu output stamps clamped "
                                "(non-monotonic sim clock)",
                  static_cast<unsigned long>(mono_.clamped()),
                  static_cast<unsigned long>(cb_count_));
    }
    pub_merged_->publish(out);
  }

  // Transform x/y/z of every point into base_link, dropping non-finite points.
  // Returns a standard 16-byte x/y/z/intensity payload + the kept point count.
  std::optional<std::pair<std::vector<std::uint8_t>, std::uint32_t>>
  transformCloud(const PointCloud2::SharedPtr msg, const Extrinsic &ext)
  {
    const Rotation rot = makeRotation(ext.roll);

    const std::uint32_t step = msg->point_step;
    const std::uint32_t n = msg->width * msg->height;
    std::int32_t offs[4] = {-1, -1, -1, -1};
    const char *names[4] = {"x", "y", "z", "intensity"};
    for (const PointField &f : msg->fields) {
      for (int i = 0; i < 4; ++i) {
        if (f.name == names[i]) {
          offs[i] = static_cast<std::int32_t>(f.offset);
        }
      }
    }
    if (step == 0 || n == 0 || offs[0] < 0 || offs[1] < 0 || offs[2] < 0) {
      return std::nullopt;
    }

    constexpr std::uint32_t OUT_STEP = 16;
    std::vector<std::uint8_t> kept_bytes;
    kept_bytes.reserve(static_cast<std::size_t>(n) * OUT_STEP);
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
      float intensity = 0.0f;
      if (offs[3] >= 0) {
        std::memcpy(&intensity, p + offs[3], 4);
      }
      // Emit [x, y, z, intensity] (16 bytes).
      const std::size_t dst = kept_bytes.size();
      kept_bytes.resize(dst + OUT_STEP);
      std::memcpy(kept_bytes.data() + dst + 0, &x, 4);
      std::memcpy(kept_bytes.data() + dst + 4, &y, 4);
      std::memcpy(kept_bytes.data() + dst + 8, &z, 4);
      std::memcpy(kept_bytes.data() + dst + 12, &intensity, 4);
    }

    return std::make_pair(std::move(kept_bytes),
                          static_cast<std::uint32_t>(kept_bytes.size() / OUT_STEP));
  }

  static std::uint64_t stampKey(const builtin_interfaces::msg::Time &t)
  {
    return static_cast<std::uint64_t>(t.sec) * 1000000000ULL + t.nanosec;
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_left_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_right_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_top_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_merged_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_left_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_right_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_top_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  Extrinsic left_ext_;
  Extrinsic right_ext_;
  Extrinsic top_ext_;
  std::unordered_map<std::string, CloudEntry> latest_;
  StampMonotonicizer mono_;
  std::uint64_t time_sync_tol_ns_{50000000ULL};  // 0.05 s
  std::uint64_t cb_count_{0};
};

}  // namespace gz_sensor_interface

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_interface::LidarSensor>());
  rclcpp::shutdown();
  return 0;
}
