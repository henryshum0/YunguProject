#pragma once

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>

namespace offboard
{

/**
 * @brief Owns all SUPER-planner-facing topic I/O for the offboard state
 * machine.
 *
 *   - in:  /planning/pos_cmd (PositionCommand, best_effort/volatile)
 *   - out: /goal_pose (PoseStamped, the single current waypoint handed to
 *          SUPER one at a time)
 *
 * Also records incoming command timestamps so the state machine can measure
 * the planner command rate (hysteresis for hand-over detection).
 */
class SuperHandler
{
public:
    explicit SuperHandler(rclcpp::Node &node,
                          const std::string &cmd_topic,
                          const std::string &goal_topic);

    // --- Incoming (SUPER -> state machine) ---
    std::shared_ptr<const mars_quadrotor_msgs::msg::PositionCommand>
    getLatestCommand() const;
    /// Drop command timestamps older than the 1 s measurement window.
    void pruneCmdStamps();
    /// Number of /planning/pos_cmd messages received in the last 1 s window.
    double cmdRateHz() const;

    // --- Outgoing (state machine -> SUPER) ---
    void publishGoal(const geometry_msgs::msg::PoseStamped &goal);

private:
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

    rclcpp::Node &node_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    mars_quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_{nullptr};
    std::deque<rclcpp::Time> cmd_stamps_;
};

}  // namespace offboard
