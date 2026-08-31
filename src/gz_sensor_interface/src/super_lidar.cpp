// super_lidar — publish a world-frame (ENU) point cloud + odom for SUPER.
//
// The input source is selected with the `cloud_source` parameter, all of which
// are produced by lidar_sensor and are ALREADY expressed in base_link:
//   "fused"      (default): /swan_gamma_v2/scan/points_fused          (merged side LiDARs)
//   "left":       /swan_gamma_v2/scan_left/points_body
//   "right":      /swan_gamma_v2/scan_right/points_body
//   "horizontal": /swan_gamma_v2/scan_horizontal/points_body
// The base_link points are then rotated/translated into the world (ENU) frame by
// the PX4 body->world pose.
//
// Topics are read from config/gz_sensor_interface.yaml (via
// sensor_sensors.launch.py):
//   in_cloud:  one of the four base_link outputs above
//   in_odom:   /fmu/out/vehicle_odometry             (PX4 NED)
//   out_cloud: /gz/point_cloud_super                 (world ENU)
//   out_odom:  /gz/odom_super                        (world ENU)
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using px4_msgs::msg::VehicleOdometry;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using nav_msgs::msg::Odometry;

namespace gz_sensor_interface
{

namespace
{

// The PX4 vehicle_odometry quaternion is NOT a plain body->NED rotation. PX4's
// own gz_bridge (GZBridge::rotateQuaternion) constructs it as:
//
//   q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD^-1
//
// where (all rotations are 180 deg, hence self-inverse):
//   q_FLU_to_FRD = (0, 1, 0, 0)             ROS FLU body -> PX4 FRD body
//   q_ENU_to_NED = (0, sqrt(2)/2, sqrt(2)/2, 0)   ENU <-> NED frame rotation
//
// Our lidar points are in the Gazebo (ROS FLU) body frame, so to place them in
// the ENU world frame we need q_FLU_to_ENU, obtained by inverting the above:
//
//   q_FLU_to_ENU = q_ENU_to_NED * q_FRD_to_NED * q_FLU_to_FRD
//                = (0, s, s, 0) * q_odom * (0, 1, 0, 0),   s = sqrt(2)/2
constexpr double K_SQRT2_2 = 0.7071067811865476;
constexpr double Q_ENU_TO_NED[4] = {0.0, K_SQRT2_2, K_SQRT2_2, 0.0};
constexpr double Q_FLU_TO_FRD[4] = {0.0, 1.0, 0.0, 0.0};

// Hamilton product q1 * q2 (each (w, x, y, z)).
void quatMul(const double *a, const double *b, double *out)
{
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// Build a sensor_msgs/PointCloud2 from xyz arrays + an rgb-ish intensity, in the
// same layout FAST-LIO/SUPER expect (fields x, y, z, intensity).
void buildCloud(const std::vector<float> &xs, const std::vector<float> &ys,
                const std::vector<float> &zs, const std::vector<float> &ints,
                const rclcpp::Time &stamp, const std::string &frame_id,
                PointCloud2 &out)
{
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;
  out.height = 1;
  out.width = static_cast<std::uint32_t>(xs.size());
  out.fields = {
    PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
  };
  out.is_bigendian = false;
  out.point_step = 16;
  out.row_step = 16 * out.width;
  out.data.resize(static_cast<std::size_t>(out.width) * out.point_step);
  out.is_dense = true;
  std::uint8_t *ptr = out.data.data();
  for (std::uint32_t i = 0; i < out.width; ++i) {
    std::memcpy(ptr + i * out.point_step + 0, &xs[i], 4);
    std::memcpy(ptr + i * out.point_step + 4, &ys[i], 4);
    std::memcpy(ptr + i * out.point_step + 8, &zs[i], 4);
    std::memcpy(ptr + i * out.point_step + 12, &ints[i], 4);
  }
}

}  // namespace

class SuperLidar : public rclcpp::Node
{
public:
  SuperLidar() : Node("super_lidar")
  {
    declare_parameter("in_cloud", "/swan_gamma_v2/scan/points_fused");
    declare_parameter("in_odom", "/fmu/out/vehicle_odometry");
    declare_parameter("out_cloud", "/gz/point_cloud_super");
    declare_parameter("out_odom", "/gz/odom_super");
    declare_parameter("world_frame", "world");
    declare_parameter("base_frame", "base_link");

    in_cloud_ = get_parameter("in_cloud").as_string();
    in_odom_ = get_parameter("in_odom").as_string();
    out_cloud_ = get_parameter("out_cloud").as_string();
    out_odom_ = get_parameter("out_odom").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    // PX4 odometry (best-effort) + the selected lidar cloud (best-effort).
    auto qos = rclcpp::QoS(5).best_effort();
    cloud_sub_ = create_subscription<PointCloud2>(
        in_cloud_, qos, [this](const PointCloud2::SharedPtr m) { onCloud(m); });
    odom_sub_ = create_subscription<VehicleOdometry>(
        in_odom_, qos, [this](const VehicleOdometry::SharedPtr m) { onPx4Odom(m); });
    cloud_pub_ = create_publisher<PointCloud2>(out_cloud_, qos);
    odom_pub_ = create_publisher<Odometry>(out_odom_, qos);
  }

private:
  // Store the latest PX4 odom as an ENU world pose (position + quaternion).
  // PX4 ENU origin = the drone launch position, matching SUPER's world frame.
  void onPx4Odom(const VehicleOdometry::SharedPtr m)
  {
    // NED -> ENU position: p_enu = (p_ned.y, p_ned.x, -p_ned.z).
    px_ = m->position[1];
    py_ = m->position[0];
    pz_ = -m->position[2];

    // Attitude: recover the ROS-FLU body -> ENU world quaternion from the PX4
    // q_FRD_to_NED message (see the derivation above the helpers):
    //   q_FLU_to_ENU = Q_ENU_TO_NED * q_odom * Q_FLU_TO_FRD
    const double q_odom[4] = {m->q[0], m->q[1], m->q[2], m->q[3]};
    double tmp[4];
    double q_enu[4];
    quatMul(Q_ENU_TO_NED, q_odom, tmp);
    quatMul(tmp, Q_FLU_TO_FRD, q_enu);
    qw_ = q_enu[0];
    qx_ = q_enu[1];
    qy_ = q_enu[2];
    qz_ = q_enu[3];
    pose_valid_ = true;

    // Republish as a world-frame (ENU) odometry for SUPER.
    Odometry out;
    out.header.stamp = now();
    out.header.frame_id = world_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = px_;
    out.pose.pose.position.y = py_;
    out.pose.pose.position.z = pz_;
    out.pose.pose.orientation.w = qw_;
    out.pose.pose.orientation.x = qx_;
    out.pose.pose.orientation.y = qy_;
    out.pose.pose.orientation.z = qz_;
    out.twist.twist.linear.x = m->velocity[1];
    out.twist.twist.linear.y = m->velocity[0];
    out.twist.twist.linear.z = -m->velocity[2];
    odom_pub_->publish(out);
  }

  void onCloud(const PointCloud2::SharedPtr msg)
  {
    if (!pose_valid_) {
      return;
    }
    // Locate x/y/z/intensity fields.
    std::int32_t offs[4] = {-1, -1, -1, -1};
    const char *names[4] = {"x", "y", "z", "intensity"};
    for (const PointField &f : msg->fields) {
      for (int i = 0; i < 4; ++i) {
        if (f.name == names[i]) {
          offs[i] = static_cast<std::int32_t>(f.offset);
        }
      }
    }
    const std::uint32_t step = msg->point_step;
    const std::uint32_t n = msg->width * msg->height;
    if (step == 0 || n == 0 || offs[0] < 0 || offs[1] < 0 || offs[2] < 0) {
      RCLCPP_WARN(get_logger(), "cloud has no x/y/z fields, skipping");
      return;
    }

    // Transform each input point into the world (ENU) frame using the latest
    // PX4 pose. The input points are ALREADY in base_link (lidar_sensor applies
    // each sensor's mounting transform), so no extra shift is needed: apply the
    // PX4 body->world rotation (r = q * p * q^-1) then add the translation.
    // ENU origin = drone launch, matching SUPER.
    std::vector<float> xs, ys, zs, ints;
    xs.reserve(n);
    ys.reserve(n);
    zs.reserve(n);
    ints.reserve(n);
    const std::uint8_t *src = msg->data.data();
    for (std::uint32_t i = 0; i < n; ++i) {
      const std::uint8_t *p = src + static_cast<std::size_t>(i) * step;
      float lx, ly, lz;
      std::memcpy(&lx, p + offs[0], 4);
      std::memcpy(&ly, p + offs[1], 4);
      std::memcpy(&lz, p + offs[2], 4);
      const float bx = lx;
      const float by = ly;
      const float bz = lz;
      // q = (qw, qx, qy, qz); rotate v = (bx,by,bz) by q: r = q * (0,v) * q^-1.
      const float w = qw_, x = qx_, y = qy_, z = qz_;
      const float vx = bx, vy = by, vz = bz;
      const float rx =
          (w * w + x * x - y * y - z * z) * vx +
          2 * (x * y - w * z) * vy +
          2 * (x * z + w * y) * vz;
      const float ry =
          2 * (x * y + w * z) * vx +
          (w * w - x * x + y * y - z * z) * vy +
          2 * (y * z - w * x) * vz;
      const float rz =
          2 * (x * z - w * y) * vx +
          2 * (y * z + w * x) * vy +
          (w * w - x * x - y * y + z * z) * vz;
      xs.push_back(rx + px_);
      ys.push_back(ry + py_);
      zs.push_back(rz + pz_);
      if (offs[3] >= 0) {
        float inten;
        std::memcpy(&inten, p + offs[3], 4);
        ints.push_back(inten);
      } else {
        ints.push_back(0.0f);
      }
    }

    PointCloud2 out;
    buildCloud(xs, ys, zs, ints, msg->header.stamp, world_frame_, out);
    cloud_pub_->publish(out);
  }

  std::string in_cloud_;
  std::string in_odom_;
  std::string out_cloud_;
  std::string out_odom_;
  std::string world_frame_;
  std::string base_frame_;

  bool pose_valid_{false};
  double px_{0.0}, py_{0.0}, pz_{0.0};
  double qw_{1.0}, qx_{0.0}, qy_{0.0}, qz_{0.0};

  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
};

}  // namespace gz_sensor_interface

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_interface::SuperLidar>());
  rclcpp::shutdown();
  return 0;
}
