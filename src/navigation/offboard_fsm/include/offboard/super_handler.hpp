#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <super_planner/msg/goal_status.hpp>
#include <super_planner/msg/planner_state.hpp>
#include <fast_lio/msg/lio_state.hpp>

namespace offboard
{

/** Owns SUPER planner I/O and command-rate tracking for the FSM. */
class SuperHandler
{
public:
    explicit SuperHandler(rclcpp::Node &node,
                          const std::string &cmd_topic,
                          const std::string &goal_topic,
                          const std::string &planner_state_topic,
                          const std::string &lio_state_topic,
                          const std::string &reset_service,
                          const std::string &goal_status_topic);

    std::shared_ptr<const mars_quadrotor_msgs::msg::PositionCommand>
    getLatestCommand() const;
    void pruneCmdStamps();
    double cmdRateHz() const;

    std::shared_ptr<const super_planner::msg::PlannerState> getPlannerState() const;
    std::shared_ptr<const fast_lio::msg::LioState> getLioState() const;
    bool isPlannerFail() const;
    bool isLioError() const;
    bool isPlannerReady() const;
    bool isPlannerWaitGoal() const;

    std::shared_ptr<const super_planner::msg::GoalStatus> getGoalStatus() const;
    void clearGoalStatus();

    void publishGoal(const geometry_msgs::msg::PoseStamped &goal);
    void requestPlannerReset(const std::function<void(bool)> &on_result);

private:
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
    void plannerStateCallback(const super_planner::msg::PlannerState::SharedPtr msg);
    void lioStateCallback(const fast_lio::msg::LioState::SharedPtr msg);
    void goalStatusCallback(const super_planner::msg::GoalStatus::SharedPtr msg);

    rclcpp::Node &node_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<super_planner::msg::PlannerState>::SharedPtr planner_state_sub_;
    rclcpp::Subscription<fast_lio::msg::LioState>::SharedPtr lio_state_sub_;
    rclcpp::Subscription<super_planner::msg::GoalStatus>::SharedPtr goal_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

    mars_quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_{nullptr};
    std::shared_ptr<super_planner::msg::PlannerState> planner_state_{nullptr};
    std::shared_ptr<fast_lio::msg::LioState> lio_state_{nullptr};
    std::shared_ptr<super_planner::msg::GoalStatus> goal_status_{nullptr};
    std::deque<rclcpp::Time> cmd_stamps_;
};

}  // namespace offboard
