// visual_tf — publish the TF tree that keeps every visualization aligned in one
// world frame anchored at the drone launch position.
//
// The visualization world frame coincides with the drone launch position, which
// is where FAST-LIO's camera_init origin and PX4's ENU local origin sit. Both
// origins overlap, but FAST-LIO's camera_init axes are aligned to the drone's
// launch heading (camera_init x == first-frame body heading), which may differ
// from the PX4 ENU world x axis. So:
//   - world -> camera_init : static, translation 0, rotation = the drone's
//                            launch heading (captured from the first PX4 odom)
//   - world -> body        : dynamic, from the PX4 ENU odom (/gz/odom_super,
//                            already in the launch-position frame)
//   - body  -> base_link   : identity (base_link == IMU origin)
//   - base_link -> lidar_link : static, the 0.16 m lidar mounting height
//
// This lets RViz (fixed frame = world) display FAST-LIO /cloud_registered and
// SUPER /gz/point_cloud_super all aligned at the drone.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::TransformStamped;

namespace visualization
{

class VisualTf : public rclcpp::Node
{
public:
  VisualTf() : Node("visual_tf")
  {
    declare_parameter("odom_topic", "/lidar_slam/odom");
    declare_parameter("world_frame", "world");
    declare_parameter("body_frame", "body");
    declare_parameter("base_frame", "base_link");
    declare_parameter("lidar_frame", "lidar_link");
    declare_parameter("camera_init_frame", "camera_init");
    declare_parameter("lidar_offset_z", 0.16);

    odom_topic_ = get_parameter("odom_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    body_frame_ = get_parameter("body_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    lidar_frame_ = get_parameter("lidar_frame").as_string();
    camera_init_frame_ = get_parameter("camera_init_frame").as_string();
    lidar_offset_z_ = get_parameter("lidar_offset_z").as_double();

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    publishStaticTfs();

    auto qos = rclcpp::QoS(5).best_effort();
    odom_sub_ = create_subscription<Odometry>(
        odom_topic_, qos, [this](const Odometry::SharedPtr m) { onOdom(m); });

    RCLCPP_INFO(get_logger(),
                "visual_tf: world=%s (drone launch origin), odom=%s, "
                "lidar_offset_z=%.2f",
                world_frame_.c_str(), odom_topic_.c_str(), lidar_offset_z_);
  }

private:
  void publishStaticTfs()
  {
    // body -> base_link: identity.
    publishStatic(body_frame_, base_frame_, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    // base_link -> lidar_link: +0.16 m up (lidar mounting height).
    publishStatic(base_frame_, lidar_frame_, 0.0, 0.0, lidar_offset_z_, 1.0, 0.0, 0.0, 0.0);
    // world -> camera_init: publish an initial identity so the frame exists;
    // once the first PX4 odom arrives we republish with the launch-heading
    // rotation (see onOdom).
    publishStatic(world_frame_, camera_init_frame_, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  }

  void publishCameraInit(const geometry_msgs::msg::Quaternion &q)
  {
    publishStatic(world_frame_, camera_init_frame_, 0.0, 0.0, 0.0,
                  q.w, q.x, q.y, q.z);
  }

  void publishStatic(const std::string &parent, const std::string &child,
                     double x, double y, double z,
                     double qw, double qx, double qy, double qz)
  {
    TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = qw;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    static_broadcaster_->sendTransform(tf);
  }

  void onOdom(const Odometry::SharedPtr m)
  {
    // The odom is already in the launch-position world frame (PX4 ENU). Forward
    // it as world -> body so RViz can follow the drone.
    TransformStamped tf;
    tf.header.stamp = m->header.stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = body_frame_;
    tf.transform.translation.x = m->pose.pose.position.x;
    tf.transform.translation.y = m->pose.pose.position.y;
    tf.transform.translation.z = m->pose.pose.position.z;
    tf.transform.rotation = m->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    // Capture the drone's launch heading from the first odom sample and anchor
    // world -> camera_init to it (camera_init x == the drone's launch heading).
    if (!camera_init_set_) {
      camera_init_set_ = true;
      publishCameraInit(m->pose.pose.orientation);
      RCLCPP_INFO(get_logger(),
                  "visual_tf: camera_init anchored to launch heading "
                  "(qw=%.3f qx=%.3f qy=%.3f qz=%.3f)",
                  m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                  m->pose.pose.orientation.y, m->pose.pose.orientation.z);
    }
  }

  std::string odom_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string camera_init_frame_;
  double lidar_offset_z_;
  bool camera_init_set_{false};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
};

}  // namespace visualization

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visualization::VisualTf>());
  rclcpp::shutdown();
  return 0;
}
