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
    // --- Parameters ---
    update_rate_ = declare_parameter("update_rate", update_rate_);
    planner_cmd_hz_ = declare_parameter("planner_cmd_hz", planner_cmd_hz_);
    planner_enter_delay_ = declare_parameter("planner_enter_delay", planner_enter_delay_);
    planner_exit_delay_ = declare_parameter("planner_exit_delay", planner_exit_delay_);
    arm_wait_ = declare_parameter("arm_wait", arm_wait_);
    default_height_ = declare_parameter("default_height", default_height_);
    takeoff_duration_ = declare_parameter("takeoff_duration", takeoff_duration_);
    landing_vel_ = declare_parameter("landing_vel", landing_vel_);
    landing_z_ = declare_parameter("landing_z", landing_z_);
    cmd_topic_ = declare_parameter("cmd_topic", cmd_topic_);
    local_pos_topic_ = declare_parameter("local_pos_topic", local_pos_topic_);
    status_topic_ = declare_parameter("status_topic", status_topic_);
    goal_topic_ = declare_parameter("goal_topic", goal_topic_);
    waypoint_marker_topic_ = declare_parameter("waypoint_marker_topic", waypoint_marker_topic_);
    waypoint_marker_rate_ = declare_parameter("waypoint_marker_rate", waypoint_marker_rate_);
    const std::string waypoint_buffer_topic =
        declare_parameter("waypoint_buffer_topic", "/waypoint_buffer");
    const double waypoint_reached_dist =
        declare_parameter("waypoint_reached_dist", 0.5);
    const double waypoint_hold_time =
        declare_parameter("waypoint_hold_time", 2.0);

    const auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));

    // --- Topic I/O handlers (PX4 and SUPER) ---
    px4_ = std::make_unique<Px4Handler>(*this, local_pos_topic_, status_topic_);
    super_ = std::make_unique<SuperHandler>(*this, cmd_topic_, goal_topic_);

    // --- Waypoint following (buffered waypoints forwarded by the goal marker
    //     node, flown one at a time via SUPER) ---
    waypoints_ = std::make_unique<WaypointHandler>(
        *this, waypoint_buffer_topic, waypoint_reached_dist, waypoint_hold_time,
        [this]() { return px4_->getLocalPosition(); },
        [this](const geometry_msgs::msg::PoseStamped &goal) {
            super_->publishGoal(goal);
        });

    // --- Waypoint buffer visualization (MarkerArray, latched + regular) ---
    vis_ = std::make_unique<VisualizationHandler>(
        *this, waypoint_marker_topic_, waypoint_marker_rate_,
        [this](std::deque<geometry_msgs::msg::PoseStamped> &buffered,
               std::optional<geometry_msgs::msg::PoseStamped> &current) {
            buffered = waypoints_->buffered();
            current = waypoints_->current();
        });

    // --- Landing service ---
    land_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/land",
        std::bind(&OffboardNode::landCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // --- Timer ---
    timer_ = create_wall_timer(update_period,
                               std::bind(&OffboardNode::timerCallback, this));

    state_enter_t_ = now();

    RCLCPP_INFO(get_logger(),
                "Offboard state machine started. Listening on %s, "
                "planner active threshold = %.1f Hz",
                cmd_topic_.c_str(), planner_cmd_hz_);
}

// ======================================================================
//  State helpers
// ======================================================================

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
        case State::INIT:         return "INIT";
        case State::ARMING:       return "ARMING";
        case State::SET_OFFBOARD: return "SET_OFFBOARD";
        case State::TAKEOFF:      return "TAKEOFF";
        case State::IDLE:         return "IDLE";
        case State::PLANNER:      return "PLANNER";
        case State::LANDING:      return "LANDING";
        case State::LANDED:       return "LANDED";
    }
    return "UNKNOWN";
}

double OffboardNode::stateElapsedSec() const
{
    return (now() - state_enter_t_).seconds();
}

// ======================================================================
//  Callbacks
// ======================================================================

void OffboardNode::landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (state_ == State::LANDING || state_ == State::LANDED) {
        res->success = false;
        res->message = "Already landing/landed";
        return;
    }
    // Remember the current hold point (or last cmd) so landing keeps xy
    if (auto local_pos = px4_->getLocalPosition()) {
        hold_x_ = local_pos->x;
        hold_y_ = local_pos->y;
        have_hold_ = true;
    }
    RCLCPP_INFO(get_logger(), "Landing requested");
    setState(State::LANDING);
    res->success = true;
    res->message = "Landing";
}

// ======================================================================
//  Setpoint helpers (built on Px4Handler::publishSetpoint)
// ======================================================================

void OffboardNode::publishHold()
{
    px4_->publishSetpoint(hold_x_, hold_y_, hold_z_, 0.0f, 0.0f, 0.0f);
}

// ======================================================================
//  Per-state handlers
// ======================================================================

void OffboardNode::handleTakeoff()
{
    const double duration = std::max(takeoff_duration_, 0.1);
    const double s = std::clamp(stateElapsedSec() / duration, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;

    // Quintic time scaling: position, velocity and acceleration are all
    // continuous, with zero velocity and acceleration at both endpoints.
    const double blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    const double blend_dot = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / duration;
    const double blend_ddot = (60.0 * s - 180.0 * s2 + 120.0 * s3)
                              / (duration * duration);

    const double dz = static_cast<double>(hold_z_ - takeoff_start_z_);
    const float z = static_cast<float>(takeoff_start_z_ + dz * blend);
    const float vz = static_cast<float>(dz * blend_dot);
    const float az = static_cast<float>(dz * blend_ddot);

    px4_->publishSetpoint(takeoff_start_x_, takeoff_start_y_, z,
                          0.0f, 0.0f, vz, NAN, 0.0f, NAN, NAN, az);

    if (s >= 1.0) {
        RCLCPP_INFO(get_logger(), "Smooth takeoff complete at z=%.2f m", hold_z_);
        setState(State::IDLE);
    }
}

// ======================================================================
//  Planner rate measurement (hysteresis)
// ======================================================================

void OffboardNode::updatePlannerActivity()
{
    // Drop stale stamps from the 1 s window, then measure the current rate.
    super_->pruneCmdStamps();
    const bool cond = super_->cmdRateHz() >= planner_cmd_hz_;
    if (cond != planner_cond_val_) {
        planner_cond_val_ = cond;
        planner_cond_t_ = now();
    }

    if (!planner_active_ && cond &&
        (now() - planner_cond_t_).seconds() > planner_enter_delay_) {
        planner_active_ = true;
        RCLCPP_INFO(get_logger(), "Planner hand-over detected (cmd rate >= %.1f Hz)",
                    planner_cmd_hz_);
    } else if (planner_active_ && !cond &&
               (now() - planner_cond_t_).seconds() > planner_exit_delay_) {
        planner_active_ = false;
        RCLCPP_INFO(get_logger(), "Planner stopped (cmd rate < %.1f Hz) → idle",
                    planner_cmd_hz_);
    }
}

// ======================================================================
//  Main state machine
// ======================================================================

void OffboardNode::timerCallback()
{
    if (state_ == State::PLANNER) {
        px4_->publishOffboardControlMode(true, true, true);
    } else {
        px4_->publishOffboardControlMode(true, false, false);
    }

    switch (state_) {
        case State::INIT:         handleInit(); break;
        case State::ARMING:       handleArming(); break;
        case State::SET_OFFBOARD: handleSetOffboard(); break;
        case State::TAKEOFF:      handleTakeoff(); break;
        case State::IDLE:         handleIdle(); break;
        case State::PLANNER:      handlePlanner(); break;
        case State::LANDING:      handleLanding(); break;
        case State::LANDED:       break;  // stop streaming; PX4 drops out of offboard
    }

    // Waypoint following runs on every state-machine tick (internally gated to
    // IDLE/PLANNER), so a waypoint reached while SUPER is still planning can
    // advance to the next one immediately.
    waypoints_->tick(state_ == State::IDLE || state_ == State::PLANNER);
}

void OffboardNode::handleInit()
{
    // Stream origin so PX4 sees the offboard stream before arming.
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);
    if (stateElapsedSec() > arm_wait_) {
        setState(State::ARMING);
    }
}

void OffboardNode::handleArming()
{
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);
    // if (stateElapsedSec() < 0.1) {
    //     arm();
    // }
    // Confirm arming from vehicle_status instead of a fixed wait.
    if (px4_->isArmed()) {
        RCLCPP_INFO(get_logger(), "Vehicle confirmed ARMED");
        setState(State::SET_OFFBOARD);
    } else {
        px4_->arm();
    }
}

void OffboardNode::handleSetOffboard()
{
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);
    if (stateElapsedSec() < 0.1) {
        px4_->setOffboardMode();
    }
    if (!px4_->isOffboard()) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Vehicle confirmed OFFBOARD mode");

    const auto local_pos = px4_->getLocalPosition();
    if (local_pos && local_pos->xy_valid && local_pos->z_valid
        && std::isfinite(local_pos->x) && std::isfinite(local_pos->y)
        && std::isfinite(local_pos->z)) {
        takeoff_start_x_ = local_pos->x;
        takeoff_start_y_ = local_pos->y;
        takeoff_start_z_ = local_pos->z;
    } else {
        takeoff_start_x_ = 0.0f;
        takeoff_start_y_ = 0.0f;
        takeoff_start_z_ = 0.0f;
    }

    hold_x_ = takeoff_start_x_;
    hold_y_ = takeoff_start_y_;
    hold_z_ = static_cast<float>(-default_height_);
    have_hold_ = true;
    RCLCPP_INFO(get_logger(),
                "Starting %.1f s smooth takeoff: z %.2f -> %.2f m",
                std::max(takeoff_duration_, 0.1), takeoff_start_z_, hold_z_);
    setState(State::TAKEOFF);
}

void OffboardNode::handleIdle()
{
    updatePlannerActivity();
    publishHold();

    if (planner_active_) {
        // Remember the current hold so we can return to it later.
        if (auto local_pos = px4_->getLocalPosition()) {
            hold_x_ = local_pos->x;
            hold_y_ = local_pos->y;
            hold_z_ = local_pos->z;
        }
        setState(State::PLANNER);
    }
}

void OffboardNode::handlePlanner()
{
    updatePlannerActivity();
    if (!planner_active_) {
        // Planner stopped → freeze at last commanded position.
        if (auto cmd = super_->getLatestCommand()) {
            frame::enuToNed(cmd->position.x,
                            cmd->position.y,
                            cmd->position.z,
                            hold_x_, hold_y_, hold_z_);
            have_hold_ = true;
        }
        // Exiting the planner means either all buffered waypoints were
        // flown (buffer already empty) or a waypoint failed; either way
        // discard any remaining waypoints and the in-flight one so the
        // drone stops at the current goal.
        const size_t pending = waypoints_->clearPending();
        if (pending > 0) {
            RCLCPP_INFO(get_logger(),
                        "Planner exited with %zu waypoint(s) unflown - "
                        "waypoint buffer cleared", pending);
        }
        setState(State::IDLE);
        return;
    }

    auto cmd = super_->getLatestCommand();
    if (!cmd) {
        publishHold();
        return;
    }

    // Forward the planner command (ENU → NED), including the trajectory's
    // acceleration as feedforward.
    const auto &c = *cmd;
    float nx, ny, nz, vx, vy, vz, ax, ay, az;
    frame::enuToNed(c.position.x, c.position.y, c.position.z, nx, ny, nz);
    frame::enuToNed(c.velocity.x, c.velocity.y, c.velocity.z, vx, vy, vz);
    frame::enuToNed(c.acceleration.x, c.acceleration.y, c.acceleration.z, ax, ay, az);

    // SUPER yaw is ENU (0 = East, CCW+); PX4 is NED (0 = North, CW+).
    // SUPER's yaw command is an unwrapped (continuous) angle and can exceed
    // [-pi, pi]; the NED yaw is wrapped into [-pi, pi] (equivalent angle) so
    // PX4 always gets a heading in range and tracks the shortest path.
    const float yaw_ned = static_cast<float>(frame::enuYawToNed(c.yaw));
    const float yawspeed_ned = static_cast<float>(frame::enuYawRateToNed(c.yaw_dot));

    px4_->publishSetpoint(nx, ny, nz, vx, vy, vz, yaw_ned, yawspeed_ned, ax, ay, az);
}

void OffboardNode::handleLanding()
{
    const float target_z = static_cast<float>(landing_z_);
    if (have_hold_) {
        px4_->publishSetpoint(hold_x_, hold_y_, target_z,
                              0.0f, 0.0f, static_cast<float>(-landing_vel_));
    } else {
        px4_->publishSetpoint(0.0f, 0.0f, target_z,
                              0.0f, 0.0f, static_cast<float>(-landing_vel_));
    }

    const auto local_pos = px4_->getLocalPosition();
    const bool on_ground = local_pos && local_pos->z > target_z;
    if (on_ground || stateElapsedSec() > 20.0) {
        px4_->disarm();
        setState(State::LANDED);
    }
}

}  // namespace offboard

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(offboard::OffboardNode)
