#include "offboard/fastlio_handler.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fastlio_handler");

    const std::string odom_topic =
        node->declare_parameter("odom_topic", "/Odometry");
    const std::string ev_topic =
        node->declare_parameter("ev_topic", "/fmu/in/vehicle_visual_odometry");

    offboard::FastLioHandler handler(*node, odom_topic, ev_topic);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
