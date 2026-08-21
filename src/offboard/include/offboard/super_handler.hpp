#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <super_planner/msg/planner_state.hpp>
#include <fast_lio/msg/lio_state.hpp>

namespace offboard
{

/**
 * @brief Owns all SUPER-planner-facing topic I/O for the offboard state
 * machine.
 *
 *   - in:  /planning/pos_cmd (PositionCommand, best_effort/volatile)
 *   - in:  fsm/planner_state (PlannerState) - high-level planner FSM state
 *   - in:  fastlio/lio_state (LioState)     - FAST-LIO odometry health
 *   - out: /goal_pose (PoseStamped, the single current waypoint handed to
 *          SUPER one at a time)
 *
 * Also records incoming command timestamps so the state machine can measure
 * the planner command rate (hysteresis for hand-over detection), and exposes
 * a reset-service client to restart the planner FSM on failure.
 */
class SuperHandler
{
public:
    explicit SuperHandler(rclcpp::Node &node,
                          const std::string &cmd_topic,
                          const std::string &goal_topic,
                          const std::string &planner_state_topic,
                          const std::string &lio_state_topic,
                          const std::string &reset_service);

    // --- Incoming (SUPER -> state machine) ---
    std::shared_ptr<const mars_quadrotor_msgs::msg::PositionCommand>
    getLatestCommand() const;
    /// Drop command timestamps older than the 1 s measurement window.
    void pruneCmdStamps();
    /// Number of /planning/pos_cmd messages received in the last 1 s window.
    double cmdRateHz() const;

    // --- Planner / LIO state (SUPER + FAST-LIO -> state machine) ---
    std::shared_ptr<const super_planner::msg::PlannerState> getPlannerState() const;
    std::shared_ptr<const fast_lio::msg::LioState> getLioState() const;
    /// True when the planner reports a continuous failure (fsm/planner_state.fail).
    bool isPlannerFail() const;
    /// True when FAST-LIO reports an error or lost its sensor stream.
    bool isLioError() const;
    /// True while the planner FSM is ready (init or waiting for a goal).
    bool isPlannerReady() const;

    // --- Outgoing (state machine -> SUPER) ---
    void publishGoal(const geometry_msgs::msg::PoseStamped &goal);
    /// Ask the planner FSM to reset back to WAIT_GOAL. Non-blocking: the
    /// service response is delivered asynchronously through `on_result`
    /// (true = reset confirmed). This must not block the executor thread,
    /// otherwise the response can never be received (single-threaded node).
    void requestPlannerReset(const std::function<void(bool)> &on_result);

private:
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
    void plannerStateCallback(const super_planner::msg::PlannerState::SharedPtr msg);
    void lioStateCallback(const fast_lio::msg::LioState::SharedPtr msg);

    rclcpp::Node &node_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<super_planner::msg::PlannerState>::SharedPtr planner_state_sub_;
    rclcpp::Subscription<fast_lio::msg::LioState>::SharedPtr lio_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

    mars_quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_{nullptr};
    std::shared_ptr<super_planner::msg::PlannerState> planner_state_{nullptr};
    std::shared_ptr<fast_lio::msg::LioState> lio_state_{nullptr};
    std::deque<rclcpp::Time> cmd_stamps_;
};

}  // namespace offboard
