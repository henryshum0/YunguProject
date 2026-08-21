#include "offboard/super_handler.hpp"

namespace offboard
{

SuperHandler::SuperHandler(rclcpp::Node &node,
                           const std::string &cmd_topic,
                           const std::string &goal_topic,
                           const std::string &planner_state_topic,
                           const std::string &lio_state_topic,
                           const std::string &reset_service)
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

    // Planner FSM state + FAST-LIO odometry health (same QoS as SUPER output).
    planner_state_sub_ = node_.create_subscription<super_planner::msg::PlannerState>(
        planner_state_topic, qos_super,
        std::bind(&SuperHandler::plannerStateCallback, this, std::placeholders::_1));
    lio_state_sub_ = node_.create_subscription<fast_lio::msg::LioState>(
        lio_state_topic, qos_super,
        std::bind(&SuperHandler::lioStateCallback, this, std::placeholders::_1));

    // Planner reset service (super_planner's ~/reset). Blocking call, so only
    // used from the PLANNER_FAIL state at low rate.
    reset_client_ = node_.create_client<std_srvs::srv::Trigger>(reset_service);
}

// ======================================================================
//  Incoming (SUPER -> state machine)
// ======================================================================

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
    // The planner is "ready" while it is idle, waiting for a goal (or still
    // initialising). Once it is planning it is busy but still healthy; only a
    // sustained FAIL means it is not ready.
    if (!planner_state_) {
        return false;
    }
    return planner_state_->init || planner_state_->wait_goal;
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
