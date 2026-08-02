#include "offboard/offboard_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<offboard::OffboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
