#include "offboard/super_handler.hpp"

namespace offboard
{

SuperHandler::SuperHandler(rclcpp::Node &node,
                           const std::string &cmd_topic,
                           const std::string &goal_topic,
                           const std::string &planner_state_topic,
                           const std::string &lio_state_topic,
                           const std::string &reset_service,
                           const std::string &goal_status_topic)
    : node_(node)
{
    auto qos_super = rclcpp::QoS(1)
        .best_effort()
        .keep_last(1)
        .durability_volatile();

    cmd_sub_ = node_.create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        cmd_topic, qos_super,
        std::bind(&SuperHandler::cmdCallback, this, std::placeholders::_1));

    goal_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic, qos_super);

    planner_state_sub_ = node_.create_subscription<super_planner::msg::PlannerState>(
        planner_state_topic, qos_super,
        std::bind(&SuperHandler::plannerStateCallback, this, std::placeholders::_1));
    lio_state_sub_ = node_.create_subscription<fast_lio::msg::LioState>(
        lio_state_topic, qos_super,
        std::bind(&SuperHandler::lioStateCallback, this, std::placeholders::_1));
    goal_status_sub_ = node_.create_subscription<super_planner::msg::GoalStatus>(
        goal_status_topic, rclcpp::QoS(10).reliable(),
        std::bind(&SuperHandler::goalStatusCallback, this, std::placeholders::_1));

    reset_client_ = node_.create_client<std_srvs::srv::Trigger>(reset_service);
}

void SuperHandler::cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    latest_cmd_ = msg;
    cmd_stamps_.push_back(node_.now());
}

void SuperHandler::plannerStateCallback(const super_planner::msg::PlannerState::SharedPtr msg)
{
    planner_state_ = msg;
}

void SuperHandler::lioStateCallback(const fast_lio::msg::LioState::SharedPtr msg)
{
    lio_state_ = msg;
}

void SuperHandler::goalStatusCallback(const super_planner::msg::GoalStatus::SharedPtr msg)
{
    goal_status_ = msg;
}

std::shared_ptr<const mars_quadrotor_msgs::msg::PositionCommand>
SuperHandler::getLatestCommand() const
{
    return latest_cmd_;
}

std::shared_ptr<const super_planner::msg::PlannerState>
SuperHandler::getPlannerState() const
{
    return planner_state_;
}

std::shared_ptr<const fast_lio::msg::LioState>
SuperHandler::getLioState() const
{
    return lio_state_;
}

bool SuperHandler::isPlannerFail() const
{
    return planner_state_ != nullptr && planner_state_->fail;
}

bool SuperHandler::isLioError() const
{
    return lio_state_ != nullptr && lio_state_->error;
}

bool SuperHandler::isPlannerReady() const
{
    if (!planner_state_) {
        return false;
    }
    return planner_state_->init || planner_state_->wait_goal || planner_state_->move;
}

bool SuperHandler::isPlannerWaitGoal() const
{
    if (!planner_state_) {
        return false;
    }
    return planner_state_->wait_goal;
}

std::shared_ptr<const super_planner::msg::GoalStatus>
SuperHandler::getGoalStatus() const
{
    return goal_status_;
}

void SuperHandler::clearGoalStatus()
{
    goal_status_.reset();
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

void SuperHandler::publishGoal(const geometry_msgs::msg::PoseStamped &goal)
{
    clearGoalStatus();
    goal_pub_->publish(goal);
}

void SuperHandler::requestPlannerReset(const std::function<void(bool)> &on_result)
{
    if (!reset_client_->service_is_ready()) {
        RCLCPP_WARN(node_.get_logger(),
                    "Planner reset service not available - will retry later");
        on_result(false);
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_client_->async_send_request(
        req,
        [this, on_result](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
                auto res = future.get();
                on_result(res->success);
            } catch (const std::exception &e) {
                RCLCPP_WARN(node_.get_logger(),
                            "Planner reset service threw: %s", e.what());
                on_result(false);
            }
        });
}

}  // namespace offboard
