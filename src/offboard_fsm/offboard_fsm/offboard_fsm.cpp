#include "offboard/offboard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "offboard/frame_conversion.hpp"

using namespace std::chrono_literals;

namespace offboard
{

OffboardNode::OffboardNode(const rclcpp::NodeOptions &options)
    : Node("offboard", options)
{
    update_rate_ = declare_parameter("update_rate", update_rate_);
    planner_cmd_hz_ = declare_parameter("planner_cmd_hz", planner_cmd_hz_);
    arm_retry_delay_ = declare_parameter("arm_retry_delay", arm_retry_delay_);
    arm_retry_max_ = declare_parameter("arm_retry_max", arm_retry_max_);
    planner_reset_delay_ = declare_parameter("planner_reset_delay", planner_reset_delay_);
    default_height_ = declare_parameter("default_height", default_height_);
    landing_vel_ = declare_parameter("landing_vel", landing_vel_);
    takeoff_vel_ = declare_parameter("takeoff_vel", takeoff_vel_);
    landing_z_ = declare_parameter("landing_z", landing_z_);
    yaw_align_thresh_ = declare_parameter("yaw_align_thresh", yaw_align_thresh_);
    cmd_topic_ = declare_parameter("cmd_topic", cmd_topic_);
    local_pos_topic_ = declare_parameter("local_pos_topic", local_pos_topic_);
    status_topic_ = declare_parameter("status_topic", status_topic_);
    land_detected_topic_ = declare_parameter("land_detected_topic", land_detected_topic_);
    goal_topic_ = declare_parameter("goal_topic", goal_topic_);
    planner_state_topic_ = declare_parameter("planner_state_topic", planner_state_topic_);
    goal_status_topic_ = declare_parameter("goal_status_topic", goal_status_topic_);
    lio_state_topic_ = declare_parameter("lio_state_topic", lio_state_topic_);
    planner_reset_service_ = declare_parameter("planner_reset_service", planner_reset_service_);
    takeoff_cmd_topic_ = declare_parameter("takeoff_cmd_topic", takeoff_cmd_topic_);
    land_cmd_topic_ = declare_parameter("land_cmd_topic", land_cmd_topic_);
    const std::string waypoint_buffer_topic =
        declare_parameter("waypoint_buffer_topic", "/waypoint_buffer");
    waypoint_reached_dist_ =
        declare_parameter("waypoint_reached_dist", waypoint_reached_dist_);
    const double waypoint_hold_time =
        declare_parameter("waypoint_hold_time", 0.0);

    const auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));

    px4_ = std::make_unique<Px4Handler>(*this, local_pos_topic_, status_topic_,
                                        land_detected_topic_);
    super_ = std::make_unique<SuperHandler>(
        *this, cmd_topic_, goal_topic_, planner_state_topic_, lio_state_topic_,
        planner_reset_service_, goal_status_topic_);

    waypoints_ = std::make_unique<WaypointHandler>(
        *this, waypoint_buffer_topic, waypoint_reached_dist_, waypoint_hold_time,
        [this]() { return px4_->getLocalPosition(); });

    land_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/land",
        std::bind(&OffboardNode::landCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    takeoff_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
        takeoff_cmd_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&OffboardNode::takeoffCmdCallback, this, std::placeholders::_1));
    land_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
        land_cmd_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&OffboardNode::landCmdCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(update_period,
                               std::bind(&OffboardNode::timerCallback, this));

    state_enter_t_ = now();
    last_arm_t_ = now();
    last_planner_reset_t_ = now();

    RCLCPP_INFO(get_logger(),
                "Offboard state machine started (planner-driven). "
                "Listening on %s, planner active threshold = %.1f Hz",
                cmd_topic_.c_str(), planner_cmd_hz_);
}

void OffboardNode::setState(State s)
{
    RCLCPP_INFO(get_logger(), "State: %s → %s", stateName(), stateNameOf(s));
    state_ = s;
    state_enter_t_ = now();
}

const char *OffboardNode::stateName() const
{
    return stateNameOf(state_);
}

const char *OffboardNode::stateNameOf(State s)
{
    switch (s) {
        case State::INIT:           return "INIT";
        case State::ARMING:         return "ARMING";
        case State::TAKEOFF:        return "TAKEOFF";
        case State::IDLE:           return "IDLE";
        case State::MOVE:           return "MOVE";
        case State::LAND:           return "LAND";
    }
    return "UNKNOWN";
}

double OffboardNode::stateElapsedSec() const
{
    return (now() - state_enter_t_).seconds();
}

void OffboardNode::takeoffCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    takeoff_requested_ = msg->data;
    if (msg->data) {
        RCLCPP_INFO(get_logger(), "Takeoff command received");
    }
}

void OffboardNode::landCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        land_requested_ = false;
        return;
    }
    if (state_ == State::INIT || state_ == State::LAND) {
        return;
    }
    land_requested_ = true;
    RCLCPP_INFO(get_logger(), "Land command received - interrupting flight");
    captureHold();
    setState(State::LAND);
}

void OffboardNode::landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (state_ == State::LAND) {
        res->success = false;
        res->message = "Already landing/failsafe";
        return;
    }
    if (state_ == State::INIT) {
        res->success = false;
        res->message = "Cannot land in this state";
        return;
    }
    captureHold();
    RCLCPP_INFO(get_logger(), "Landing requested");
    setState(State::LAND);
    res->success = true;
    res->message = "Landing";
}

void OffboardNode::publishHold()
{
    px4_->publishSetpoint(hold_x_, hold_y_, hold_z_, 0.0f, 0.0f, 0.0f);
}

void OffboardNode::publishIdleHold()
{
    float yaw_ned = 0.0f;
    const auto goal = headingTarget();
    const auto local_pos = px4_->getLocalPosition();
    if (goal && local_pos && local_pos->xy_valid) {
        const double enu_x = local_pos->y;
        const double enu_y = local_pos->x;
        const double bearing = std::atan2(
            goal->pose.position.y - enu_y, goal->pose.position.x - enu_x);
        yaw_ned = static_cast<float>(
            std::remainder(frame::kPiHalf - bearing, frame::kTwoPi));
    }
    px4_->publishSetpoint(hold_x_, hold_y_, hold_z_,
                          0.0f, 0.0f, 0.0f, yaw_ned, 0.0f);
}

void OffboardNode::captureHold()
{
    if (const auto local_pos = px4_->getLocalPosition()) {
        hold_x_ = local_pos->x;
        hold_y_ = local_pos->y;
        hold_z_ = local_pos->z;
        have_hold_ = true;
    }
}

void OffboardNode::publishGoalToPlanner(const geometry_msgs::msg::PoseStamped &goal)
{
    active_goal_ = goal;
    super_->publishGoal(goal);
}

bool OffboardNode::publishCurrentGoal()
{
    if (!waypoints_->advanceToNext()) {
        return false;
    }
    const auto goal = waypoints_->currentGoal();
    if (!goal) {
        return false;
    }
    publishGoalToPlanner(*goal);
    RCLCPP_INFO(get_logger(), "Goal forwarded to SUPER: (%.2f, %.2f, %.2f)",
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    return true;
}

bool OffboardNode::systemReady() const
{
    if (!px4_->hasValidPosition()) {
        return false;
    }
    if (super_->isLioError()) {
        return false;
    }
    if (!super_->isPlannerReady()) {
        return false;
    }
    if (!px4_->isDisarmed()) {
        return false;
    }
    if (!px4_->isLanded()) {
        return false;
    }
    return true;
}

std::optional<geometry_msgs::msg::PoseStamped> OffboardNode::headingTarget() const
{
    const auto cur = waypoints_->currentGoal();
    if (cur.has_value() && !waypoints_->hasReachedCurrent()) {
        return cur;
    }
    return waypoints_->nextGoal();
}

bool OffboardNode::headingOk() const
{
    const auto goal = headingTarget();
    if (!goal) {
        return false;
    }
    const auto local_pos = px4_->getLocalPosition();
    if (!local_pos || !local_pos->xy_valid) {
        return false;
    }

    const double enu_x = local_pos->y;
    const double enu_y = local_pos->x;
    const double g_dx = goal->pose.position.x - enu_x;
    const double g_dy = goal->pose.position.y - enu_y;
    const double bearing = std::atan2(g_dy, g_dx);  // ENU, CCW from East

    const double yaw_ned = static_cast<double>(local_pos->heading);
    const double yaw_enu = std::remainder(frame::kPiHalf - yaw_ned, frame::kTwoPi);
    const double delta = std::remainder(bearing - yaw_enu, frame::kTwoPi);
    return std::abs(delta) <= yaw_align_thresh_;
}

void OffboardNode::updatePlannerActivity()
{
    super_->pruneCmdStamps();
    const bool cond = super_->cmdRateHz() >= planner_cmd_hz_;
    if (cond != planner_cond_val_) {
        planner_cond_val_ = cond;
        planner_cond_t_ = now();
    }
    planner_active_ = cond;
}

void OffboardNode::restartPlanner()
{
    if (planner_reset_in_flight_) {
        return;
    }
    RCLCPP_INFO(get_logger(), "Restarting planner...");
    planner_reset_in_flight_ = true;
    last_planner_reset_t_ = now();
    super_->requestPlannerReset([this](bool ok) {
        planner_reset_in_flight_ = false;
        if (!ok) {
            RCLCPP_WARN(get_logger(), "Planner restart failed; will retry");
        }
        else {
            RCLCPP_INFO(get_logger(), "Planner is reset to WAIT_GOAL");
        }
    });
}

void OffboardNode::timerCallback()
{
    if (state_ == State::MOVE || state_ == State::TAKEOFF ||
        state_ == State::LAND) {
        px4_->publishOffboardControlMode(true, true, true);
    } else {
        px4_->publishOffboardControlMode(true, false, false);
    }

    switch (state_) {
        case State::INIT:           handleInit(); break;
        case State::ARMING:         handleArming(); break;
        case State::TAKEOFF:        handleTakeoff(); break;
        case State::IDLE:           handleIdle(); break;
        case State::MOVE:           handleMove(); break;
        case State::LAND:           handleLand(); break;
    }

    waypoints_->tick(state_ == State::IDLE || state_ == State::MOVE ||
                     state_ == State::TAKEOFF);
}

}  // namespace offboard

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(offboard::OffboardNode)
