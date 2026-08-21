// tf_bridge — Publish the TF tree needed to visualize the LiDAR in RViz.
//
// The ros_gz_bridge parameter_bridge only forwards messages — it does NOT
// publish TF. Without a transform from RViz's fixed frame to the cloud frame
// (lidar_link) nothing is displayed. This node provides:
//   world -> base_link      (dynamic, from the bridged /odom)
//   base_link -> lidar_link (static; lidar is 0.16 m above base_link)
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>

using nav_msgs::msg::Odometry;

namespace lidar_bridge
{

class TfBridge : public rclcpp::Node
{
public:
  TfBridge() : Node("gz_tf_bridge")
  {
    declare_parameter("odom_topic", "/lidar_slam/odom");
    declare_parameter("world_frame", "world");
    declare_parameter("base_frame", "base_link");
    declare_parameter("lidar_frame", "lidar_link");
    declare_parameter("lidar_offset_x", 0.0);
    declare_parameter("lidar_offset_y", 0.0);
    declare_parameter("lidar_offset_z", 0.16);

    const std::string odom_topic = get_parameter("odom_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    const std::string lidar_frame = get_parameter("lidar_frame").as_string();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Static base_link -> lidar_link.
    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = now();
    static_tf.header.frame_id = base_frame_;
    static_tf.child_frame_id = lidar_frame;
    static_tf.transform.translation.x = get_parameter("lidar_offset_x").as_double();
    static_tf.transform.translation.y = get_parameter("lidar_offset_y").as_double();
    static_tf.transform.translation.z = get_parameter("lidar_offset_z").as_double();
    static_tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_tf);

    // Dynamic world -> base_link from odometry. /lidar_slam/odom is published
    // best_effort/volatile by super_bridge; subscribe BEST_EFFORT to avoid an
    // incompatible-QoS warning and a never-updating TF.
    auto qos = rclcpp::QoS(10).best_effort().keep_last(10);
    odom_sub_ = create_subscription<Odometry>(
        odom_topic, qos,
        [this](const Odometry::SharedPtr msg) { onOdom(msg); });

    RCLCPP_INFO(get_logger(),
                "tf_bridge: %s -> %s (from %s), static %s -> %s (z=%.2f m)",
                world_frame_.c_str(), base_frame_.c_str(), odom_topic.c_str(),
                base_frame_.c_str(), lidar_frame.c_str(),
                get_parameter("lidar_offset_z").as_double());
  }

private:
  void onOdom(const Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = world_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  std::string world_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
};

}  // namespace lidar_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_bridge::TfBridge>());
  rclcpp::shutdown();
  return 0;
}
