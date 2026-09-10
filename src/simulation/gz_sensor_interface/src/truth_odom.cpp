// truth_odom — republish the Gazebo ground-truth odometry under a stable topic.
//
// Gazebo publishes the truth odometry on /odom (frame world). This node
// republishes it as /gz/ground_truth/odom so consumers (e.g. the visualization
// layer's gt_path) can rely on a fixed topic name, independent of the Gazebo
// bridge config.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

using nav_msgs::msg::Odometry;

namespace gz_sensor_interface
{

class TruthOdom : public rclcpp::Node
{
public:
  TruthOdom() : Node("truth_odom")
  {
    declare_parameter("input_topic", "/odom");
    declare_parameter("output_topic", "/gz/ground_truth/odom");
    const std::string in = get_parameter("input_topic").as_string();
    const std::string out = get_parameter("output_topic").as_string();

    auto qos = rclcpp::QoS(5).best_effort();
    sub_ = create_subscription<Odometry>(
        in, qos, [this](const Odometry::SharedPtr m) { onOdom(m); });
    pub_ = create_publisher<Odometry>(out, qos);
    RCLCPP_INFO(get_logger(), "truth_odom: %s -> %s", in.c_str(), out.c_str());
  }

private:
  void onOdom(const Odometry::SharedPtr m)
  {
    pub_->publish(*m);
  }

  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_;
};

}  // namespace gz_sensor_interface

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_interface::TruthOdom>());
  rclcpp::shutdown();
  return 0;
}
