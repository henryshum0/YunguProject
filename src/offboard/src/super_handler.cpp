#include "offboard/super_handler.hpp"

namespace offboard
{

SuperHandler::SuperHandler(rclcpp::Node &node,
                           const std::string &cmd_topic,
                           const std::string &goal_topic)
    : node_(node)
{
    // SUPER publishes best_effort/volatile/keep_last(1) — must match exactly
    auto qos_super = rclcpp::QoS(1)
        .best_effort()
        .keep_last(1)
        .durability_volatile();

    cmd_sub_ = node_.create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        cmd_topic, qos_super,
        std::bind(&SuperHandler::cmdCallback, this, std::placeholders::_1));

    // Waypoint goal publisher (current waypoint -> SUPER, one at a time).
    // SUPER subscribes with best_effort/volatile, so match that QoS exactly.
    goal_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic, qos_super);
}

// ======================================================================
//  Incoming (SUPER -> state machine)
// ======================================================================

void SuperHandler::cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    latest_cmd_ = msg;
    cmd_stamps_.push_back(node_.now());
}

std::shared_ptr<const mars_quadrotor_msgs::msg::PositionCommand>
SuperHandler::getLatestCommand() const
{
    return latest_cmd_;
}

void SuperHandler::pruneCmdStamps()
{
    const rclcpp::Time cutoff = node_.now() - rclcpp::Duration(1, 0);
    while (!cmd_stamps_.empty() && cmd_stamps_.front() < cutoff) {
        cmd_stamps_.pop_front();
    }
}

double SuperHandler::cmdRateHz() const
{
    // Count stamps inside the last 1 s window (without mutating).
    const rclcpp::Time cutoff = node_.now() - rclcpp::Duration(1, 0);
    size_t n = 0;
    for (auto it = cmd_stamps_.rbegin(); it != cmd_stamps_.rend(); ++it) {
        if (*it < cutoff) {
            break;
        }
        ++n;
    }
    return static_cast<double>(n);
}

// ======================================================================
//  Outgoing (state machine -> SUPER)
// ======================================================================

void SuperHandler::publishGoal(const geometry_msgs::msg::PoseStamped &goal)
{
    goal_pub_->publish(goal);
}

}  // namespace offboard
