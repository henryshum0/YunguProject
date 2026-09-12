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
    arm_wait_ = declare_parameter("arm_wait", arm_wait_);
    arm_retry_delay_ = declare_parameter("arm_retry_delay", arm_retry_delay_);
    arm_retry_max_ = declare_parameter("arm_retry_max", arm_retry_max_);
    land_retry_delay_ = declare_parameter("land_retry_delay", land_retry_delay_);
    disarm_retry_delay_ =
        declare_parameter("disarm_retry_delay", disarm_retry_delay_);
    planner_reset_delay_ = declare_parameter("planner_reset_delay", planner_reset_delay_);
    default_height_ = declare_parameter("default_height", default_height_);
    takeoff_vel_ = declare_parameter("takeoff_vel", takeoff_vel_);
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
    takeoff_service_ = declare_parameter("takeoff_service", takeoff_service_);
    land_service_ = declare_parameter("land_service", land_service_);
    waypoint_queue_service_ =
        declare_parameter("waypoint_queue_service", waypoint_queue_service_);
    clear_waypoints_service_ =
        declare_parameter("clear_waypoints_service", clear_waypoints_service_);
    waypoint_queue_status_topic_ =
        declare_parameter("waypoint_queue_status_topic", waypoint_queue_status_topic_);
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
        *this, waypoint_reached_dist_, waypoint_hold_time,
        [this]() { return px4_->getLocalPosition(); });

    takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
        takeoff_service_,
        std::bind(&OffboardNode::takeoffCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    land_srv_ = create_service<std_srvs::srv::Trigger>(
        land_service_,
        std::bind(&OffboardNode::landCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    waypoint_queue_srv_ = create_service<offboard_fsm::srv::QueueWaypoints>(
        waypoint_queue_service_,
        std::bind(&OffboardNode::queueWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    clear_waypoints_srv_ = create_service<offboard_fsm::srv::ClearWaypoints>(
        clear_waypoints_service_,
        std::bind(&OffboardNode::clearWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    waypoint_queue_status_pub_ = create_publisher<nav_msgs::msg::Path>(
        waypoint_queue_status_topic_, rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(update_period,
                               std::bind(&OffboardNode::timerCallback, this));

    state_enter_t_ = now();
    last_arm_t_ = now();
    last_planner_reset_t_ = now();
    publishWaypointQueue();

    RCLCPP_INFO(get_logger(),
                "Offboard state machine started (planner-driven). "
                "Listening on %s, planner active threshold = %.1f Hz",
                cmd_topic_.c_str(), planner_cmd_hz_);
}

void OffboardNode::queueWaypointsCallback(
    const std::shared_ptr<offboard_fsm::srv::QueueWaypoints::Request> req,
    std::shared_ptr<offboard_fsm::srv::QueueWaypoints::Response> res)
{
    if (req->waypoints.empty()) {
        res->success = false;
        res->message = "waypoint queue request must contain at least one waypoint";
        res->queued_count = 0;
        return;
    }
    res->queued_count = static_cast<uint32_t>(waypoints_->enqueue(req->waypoints));
    publishWaypointQueueIfChanged();
    res->success = true;
    res->message = "queued " + std::to_string(res->queued_count) + " waypoint(s)";
}

void OffboardNode::clearWaypointsCallback(
    const std::shared_ptr<offboard_fsm::srv::ClearWaypoints::Request> /*req*/,
    std::shared_ptr<offboard_fsm::srv::ClearWaypoints::Response> res)
{
    res->cleared_count = static_cast<uint32_t>(waypoints_->clearPending());
    active_goal_ = geometry_msgs::msg::PoseStamped();
    super_->clearGoalStatus();
    stuck_recovery_attempted_ = false;

    if (state_ == State::MOVE) {
        captureHold();
        setState(State::IDLE);
        restartPlanner();
    }

    res->success = true;
    res->message = "cleared " + std::to_string(res->cleared_count) + " waypoint(s)";
    publishWaypointQueueIfChanged();
}

void OffboardNode::setState(State s)
{
    RCLCPP_INFO(get_logger(), "State: %s → %s", stateName(), stateNameOf(s));
    state_ = s;
    state_enter_t_ = now();
    if (s == State::INIT) {
        // A new takeoff cycle must establish a fresh, stable offboard stream.
        offboard_ready_ = false;
        have_takeoff_goal_ = false;
    }
    if (s == State::LAND) {
        // Landing may be interrupted and entered again; never reuse stale
        // native-land or disarm attempts from a prior touchdown.
        land_command_requested_ = false;
        disarm_requested_ = false;
    }
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

void OffboardNode::takeoffCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (state_ != State::INIT) {
        res->success = false;
        res->message = std::string("takeoff is only accepted in INIT (currently ") +
                       stateName() + ")";
        return;
    }

    const bool already_requested = takeoff_requested_;
    // Latch the request until normal readiness and arming checks complete.
    // A client therefore submits it once even when PX4 pre-arm checks take time.
    takeoff_requested_ = true;
    RCLCPP_INFO(get_logger(), "Takeoff service request accepted");
    res->success = true;
    res->message = already_requested
                       ? "takeoff request already accepted; waiting for readiness"
                       : "takeoff accepted; waiting for normal readiness and arming checks";
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
    RCLCPP_INFO(get_logger(), "Landing service request accepted");
    setState(State::LAND);
    res->success = true;
    res->message = "landing accepted; PX4 touchdown and disarm are asynchronous";
}

void OffboardNode::publishHold()
{
    px4_->publishSetpoint(hold_x_, hold_y_, hold_z_,
                          0.0f, 0.0f, 0.0f, hold_yaw_, 0.0f);
}

void OffboardNode::publishIdleHold()
{
    // Hold the yaw captured with the position after a completed waypoint or
    // any other transition into hold. Only pre-align when another waypoint is
    // actually queued for execution.
    float yaw_ned = hold_yaw_;
    const auto goal = headingTarget();
    const auto local_pos = px4_->getLocalPosition();
    if (waypoints_->hasPendingGoal() && goal && local_pos && local_pos->xy_valid) {
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
        if (std::isfinite(local_pos->heading)) {
            hold_yaw_ = local_pos->heading;
        }
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
    publishWaypointQueueIfChanged();
    RCLCPP_INFO(get_logger(), "Goal forwarded to SUPER: (%.2f, %.2f, %.2f)",
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    return true;
}

void OffboardNode::publishWaypointQueue()
{
    nav_msgs::msg::Path snapshot;
    snapshot.header.stamp = now();

    if (const auto current = waypoints_->currentGoal()) {
        snapshot.poses.push_back(*current);
    }
    for (const auto &waypoint : waypoints_->buffered()) {
        snapshot.poses.push_back(waypoint);
    }
    if (!snapshot.poses.empty()) {
        snapshot.header.frame_id = snapshot.poses.front().header.frame_id;
    }
    waypoint_queue_status_pub_->publish(snapshot);
    published_waypoint_revision_ = waypoints_->revision();
}

void OffboardNode::publishWaypointQueueIfChanged()
{
    if (published_waypoint_revision_ != waypoints_->revision()) {
        publishWaypointQueue();
    }
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
    if (state_ == State::MOVE || state_ == State::TAKEOFF) {
        px4_->publishOffboardControlMode(true, true, true);
    } else if (state_ != State::LAND) {
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
    publishWaypointQueueIfChanged();
}

}  // namespace offboard

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(offboard::OffboardNode)
