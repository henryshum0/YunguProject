#include "super_px4_bridge/super_px4_bridge.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<super_px4_bridge::SuperPx4Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
