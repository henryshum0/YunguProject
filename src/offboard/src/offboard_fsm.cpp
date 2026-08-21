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
    arm_retry_delay_ = declare_parameter("arm_retry_delay", arm_retry_delay_);
    arm_retry_max_ = declare_parameter("arm_retry_max", arm_retry_max_);
    planner_fail_retry_max_ = declare_parameter("planner_fail_retry_max", planner_fail_retry_max_);
    planner_reset_delay_ = declare_parameter("planner_reset_delay", planner_reset_delay_);
    default_height_ = declare_parameter("default_height", default_height_);
    landing_vel_ = declare_parameter("landing_vel", landing_vel_);
    takeoff_vel_ = declare_parameter("takeoff_vel", takeoff_vel_);
    landing_z_ = declare_parameter("landing_z", landing_z_);
    yaw_align_thresh_ = declare_parameter("yaw_align_thresh", yaw_align_thresh_);
    cmd_topic_ = declare_parameter("cmd_topic", cmd_topic_);
    local_pos_topic_ = declare_parameter("local_pos_topic", local_pos_topic_);
    status_topic_ = declare_parameter("status_topic", status_topic_);
    goal_topic_ = declare_parameter("goal_topic", goal_topic_);
    planner_state_topic_ = declare_parameter("planner_state_topic", planner_state_topic_);
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

    // --- Topic I/O handlers (PX4 and SUPER) ---
    px4_ = std::make_unique<Px4Handler>(*this, local_pos_topic_, status_topic_);
    super_ = std::make_unique<SuperHandler>(
        *this, cmd_topic_, goal_topic_, planner_state_topic_, lio_state_topic_,
        planner_reset_service_);

    // --- Waypoint following (buffered waypoints fed by the goal marker node;
    //     the state machine is the sole /goal_pose publisher and forwards the
    //     current waypoint to SUPER when in MOVE) ---
    waypoints_ = std::make_unique<WaypointHandler>(
        *this, waypoint_buffer_topic, waypoint_reached_dist_, waypoint_hold_time,
        [this]() { return px4_->getLocalPosition(); });

    // --- Landing service (kept for backward compatibility) ---
    land_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/land",
        std::bind(&OffboardNode::landCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // --- User takeoff / land command topics ---
    takeoff_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
        takeoff_cmd_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&OffboardNode::takeoffCmdCallback, this, std::placeholders::_1));
    land_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
        land_cmd_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&OffboardNode::landCmdCallback, this, std::placeholders::_1));

    // --- Timer ---
    timer_ = create_wall_timer(update_period,
                               std::bind(&OffboardNode::timerCallback, this));

    state_enter_t_ = now();
    // Initialise the time-based cooldown stamps to the current ROS time so
    // the first subtraction (now() - last_*) uses a matching time source.
    last_arm_t_ = now();
    last_planner_reset_t_ = now();

    RCLCPP_INFO(get_logger(),
                "Offboard state machine started (planner-driven). "
                "Listening on %s, planner active threshold = %.1f Hz",
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
        case State::INIT:           return "INIT";
        case State::ARMING:         return "ARMING";
        case State::TAKEOFF:        return "TAKEOFF";
        case State::IDLE:           return "IDLE";
        case State::MOVE:           return "MOVE";
        case State::PLANNER_FAIL:   return "PLANNER_FAIL";
        case State::FAILSAFE:       return "FAILSAFE";
        case State::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case State::LAND:           return "LAND";
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
    // Land command is honoured in every state except INIT / PLANNER_FAIL /
    // FAILSAFE (those are handled by their own handlers). Already-landing is a
    // no-op.
    if (state_ == State::INIT || state_ == State::PLANNER_FAIL ||
        state_ == State::FAILSAFE || state_ == State::LAND) {
        return;
    }
    land_requested_ = true;
    RCLCPP_INFO(get_logger(), "Land command received - interrupting flight");
    // Remember the current hold so LAND keeps xy.
    if (auto local_pos = px4_->getLocalPosition()) {
        hold_x_ = local_pos->x;
        hold_y_ = local_pos->y;
        have_hold_ = true;
    }
    setState(State::LAND);
}

void OffboardNode::landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (state_ == State::LAND || state_ == State::FAILSAFE) {
        res->success = false;
        res->message = "Already landing/failsafe";
        return;
    }
    if (state_ == State::INIT || state_ == State::PLANNER_FAIL) {
        res->success = false;
        res->message = "Cannot land in this state";
        return;
    }
    if (auto local_pos = px4_->getLocalPosition()) {
        hold_x_ = local_pos->x;
        hold_y_ = local_pos->y;
        have_hold_ = true;
    }
    RCLCPP_INFO(get_logger(), "Landing requested");
    setState(State::LAND);
    res->success = true;
    res->message = "Landing";
}

// ======================================================================
//  Setpoint / goal helpers
// ======================================================================

void OffboardNode::publishHold()
{
    px4_->publishSetpoint(hold_x_, hold_y_, hold_z_, 0.0f, 0.0f, 0.0f);
}

void OffboardNode::publishIdleHold()
{
    // Compute the desired heading (NED yaw) toward the current buffered goal
    // so the drone faces the next waypoint while hovering in IDLE.
    float yaw_ned = 0.0f;
    const auto goal = waypoints_->currentGoal();
    const auto local_pos = px4_->getLocalPosition();
    if (goal && local_pos && local_pos->xy_valid) {
        // Current position ENU (from NED): (x=ned.y, y=ned.x).
        const double enu_x = local_pos->y;
        const double enu_y = local_pos->x;
        const double bearing = std::atan2(
            goal->pose.position.y - enu_y, goal->pose.position.x - enu_x);
        // SUPER uses ENU yaw; PX4 wants NED yaw (yaw_ned = pi/2 - yaw_enu).
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

bool OffboardNode::publishCurrentGoalIfChanged()
{
    const auto goal = waypoints_->currentGoal();
    if (!goal) {
        return false;
    }
    const auto &q = goal->pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double x = goal->pose.position.x;
    const double y = goal->pose.position.y;
    const double z = goal->pose.position.z;

    // Publish when the buffered current goal differs from the last one sent.
    if (!goal_sent_ ||
        std::abs(x - sent_goal_x_) > 1e-3 ||
        std::abs(y - sent_goal_y_) > 1e-3 ||
        std::abs(z - sent_goal_z_) > 1e-3 ||
        std::abs(yaw - sent_goal_yaw_) > 1e-3) {
        publishGoalToPlanner(*goal);
        goal_sent_ = true;
        sent_goal_x_ = x;
        sent_goal_y_ = y;
        sent_goal_z_ = z;
        sent_goal_yaw_ = yaw;
        RCLCPP_INFO(get_logger(), "Goal forwarded to SUPER: (%.2f, %.2f, %.2f)",
                    x, y, z);
        return true;
    }
    return false;
}

// ======================================================================
//  System readiness / conditions
// ======================================================================

bool OffboardNode::systemReady() const
{
    // Odometry must be present and valid.
    const auto local_pos = px4_->getLocalPosition();
    if (!local_pos || !local_pos->xy_valid || !local_pos->z_valid ||
        !std::isfinite(local_pos->x) || !std::isfinite(local_pos->y) ||
        !std::isfinite(local_pos->z)) {
        return false;
    }
    // FAST-LIO must be running (EKF initialised, sensors flowing).
    if (super_->isLioError()) {
        return false;
    }
    // The planner must be ready (init or waiting for a goal).
    if (!super_->isPlannerReady()) {
        return false;
    }
    // The vehicle must be disarmed and near the ground (landed).
    if (!px4_->isDisarmed()) {
        return false;
    }
    // NED z > -0.5 means within half a metre of the ground (z is negative up).
    if (local_pos->z < -0.5) {
        return false;
    }
    return true;
}

bool OffboardNode::computeCanMove() const
{
    // can_move_ requires a current (in-flight) goal and a local position.
    // The current goal is the one the drone is being flown toward (in IDLE,
    // the previous goal has already been reached and WaypointHandler advanced
    // to the next buffered one).
    const auto &goal = waypoints_->currentGoal();
    if (!goal) {
        return false;
    }
    const auto local_pos = px4_->getLocalPosition();
    if (!local_pos || !local_pos->xy_valid) {
        return false;
    }

    // The current goal must be "in front" of the drone: the heading should
    // point toward it (within yaw_align_thresh_). This is the "for now"
    // can_move_ gate: reached a goal (implicitly, being in IDLE) and the
    // heading points toward the next goal.
    // PX4 local position is NED; convert to ENU (planner convention).
    const double enu_x = local_pos->y;
    const double enu_y = local_pos->x;
    const double g_dx = goal->pose.position.x - enu_x;
    const double g_dy = goal->pose.position.y - enu_y;
    const double bearing = std::atan2(g_dy, g_dx);  // ENU, CCW from East

    // Current heading. PX4 reports `heading` as the NED yaw (0 = North, CW+).
    // SUPER uses ENU yaw (0 = East, CCW+), related by yaw_enu = pi/2 - yaw_ned.
    const double yaw_ned = static_cast<double>(local_pos->heading);
    const double yaw_enu = std::remainder(frame::kPiHalf - yaw_ned, frame::kTwoPi);
    const double delta = std::remainder(bearing - yaw_enu, frame::kTwoPi);
    return std::abs(delta) <= yaw_align_thresh_;
}

void OffboardNode::updatePlannerActivity()
{
    // Drop stale stamps from the 1 s window, then measure the current rate.
    super_->pruneCmdStamps();
    const bool cond = super_->cmdRateHz() >= planner_cmd_hz_;
    if (cond != planner_cond_val_) {
        planner_cond_val_ = cond;
        planner_cond_t_ = now();
    }
    // The planner is "active" while its command stream is flowing. A drop
    // below the threshold (plan rate drop) is used to leave MOVE.
    planner_active_ = cond;

    // (Re)compute the gate. "can move" when the current goal was reached and
    // the heading points toward the next goal.
    can_move_ = computeCanMove();
}

// ======================================================================
//  Planner failure handling
// ======================================================================

void OffboardNode::enterPlannerFail()
{
    ++planner_fail_count_;
    if (planner_fail_count_ >= planner_fail_retry_max_) {
        RCLCPP_ERROR(get_logger(),
                     "Planner failed %d consecutive times - entering FAILSAFE",
                     planner_fail_count_);
        setState(State::FAILSAFE);
        return;
    }
    fail_resume_state_ = state_;
    RCLCPP_WARN(get_logger(), "Planner failure #%d detected - entering PLANNER_FAIL",
                planner_fail_count_);
    setState(State::PLANNER_FAIL);
}

void OffboardNode::restartPlanner()
{
    RCLCPP_INFO(get_logger(), "Restarting planner (reset service)...");
    // Non-blocking request: the response callback re-publishes the goal and
    // resumes the FSM. This avoids blocking the single-threaded executor,
    // which would otherwise prevent the service response from ever arriving.
    super_->requestPlannerReset([this](bool ok) {
        if (!ok) {
            RCLCPP_WARN(get_logger(), "Planner reset failed - will retry after cooldown");
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Planner reset acknowledged - re-publishing goal and resuming %s",
                    stateNameOf(fail_resume_state_));
        // Re-publish the goal so the (now-reset) planner re-plans to it.
        publishGoalToPlanner(active_goal_);
        setState(fail_resume_state_);
    });
}

void OffboardNode::resetPlanner()
{
    RCLCPP_INFO(get_logger(), "Resetting planner (goal reached)...");
    // Fire-and-forget reset to WAIT_GOAL so the planner clears any residual
    // FAIL state before the next goal. No goal is re-published and no state is
    // resumed here.
    super_->requestPlannerReset([this](bool ok) {
        if (!ok) {
            RCLCPP_WARN(get_logger(), "Planner reset (goal reached) failed");
        }
    });
}

// ======================================================================
//  Main state machine
// ======================================================================

void OffboardNode::timerCallback()
{
    // Control mode flags: position control in planner/flight states, else a
    // plain position hold.
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
        case State::PLANNER_FAIL:   handlePlannerFail(); break;
        case State::FAILSAFE:       handleFailsafe(); break;
        case State::EMERGENCY_STOP: handleEmergencyStop(); break;
        case State::LAND:           handleLand(); break;
    }

    // Waypoint following runs during the flight states (internally gated).
    // In IDLE/MOVE the buffered waypoints are advanced one at a time.
    waypoints_->tick(state_ == State::IDLE || state_ == State::MOVE ||
                     state_ == State::TAKEOFF);
}

void OffboardNode::handleInit()
{
    // Stream origin so PX4 sees the offboard stream (and so OFFBOARD mode is
    // accepted) before we arm.
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);

    // Critical failure during INIT: FAST-LIO error or unrecoverable planner
    // failure.
    if (super_->isLioError() ||
        (super_->getPlannerState() && super_->isPlannerFail())) {
        RCLCPP_ERROR(get_logger(), "Critical failure during INIT - emergency stop");
        setState(State::EMERGENCY_STOP);
        return;
    }

    // If the vehicle is already airborne in OFFBOARD mode (e.g. the FSM was
    // restarted mid-flight) and odometry + planner are ready, resume directly
    // in IDLE instead of waiting for a takeoff command / going through
    // ARMING->TAKEOFF.
    if (px4_->isOffboard() && super_->isPlannerReady()) {
        const auto local_pos = px4_->getLocalPosition();
        const bool in_air = local_pos && local_pos->xy_valid &&
                            local_pos->z_valid && local_pos->z < -0.5;
        if (in_air) {
            RCLCPP_INFO(get_logger(),
                        "Vehicle already OFFBOARD and airborne - resuming in IDLE");
            captureHold();
            planner_fail_count_ = 0;
            setState(State::IDLE);
            return;
        }
    }

    // Wait until the whole system is ready (odom + fastlio + planner +
    // landed & disarmed), then arm OFFBOARD mode and await takeoff.
    if (!systemReady()) {
        return;
    }

    if (!px4_->isOffboard()) {
        px4_->setOffboardMode();
        return;
    }

    if (takeoff_requested_) {
        RCLCPP_INFO(get_logger(), "Takeoff requested - entering ARMING");
        takeoff_requested_ = false;
        arm_retry_count_ = 0;
        setState(State::ARMING);
    }
}

void OffboardNode::handleArming()
{
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);

    // Heartbeat loss (armed but no longer healthy) -> back to INIT.
    if (super_->isLioError() || !super_->isPlannerReady()) {
        RCLCPP_WARN(get_logger(), "Heartbeat/link failed during ARMING - back to INIT");
        setState(State::INIT);
        return;
    }

    if (px4_->isArmed()) {
        RCLCPP_INFO(get_logger(), "Vehicle confirmed ARMED");
        // The takeoff origin is captured in handleTakeoff (direct PX4 climb).
        have_takeoff_goal_ = false;
        setState(State::TAKEOFF);
        return;
    }

    // Retry arming with a delay between attempts. Guard against a
    // default-constructed stamp (system-clock source) which cannot be
    // subtracted from the ROS-time `now()`.
    const double elapsed = last_arm_t_.nanoseconds() != 0
                               ? (now() - last_arm_t_).seconds()
                               : arm_retry_delay_;
    if (elapsed >= arm_retry_delay_) {
        px4_->arm();
        last_arm_t_ = now();
        ++arm_retry_count_;
        RCLCPP_INFO(get_logger(), "Arm attempt #%d/%d",
                    arm_retry_count_, arm_retry_max_);
        if (arm_retry_count_ >= arm_retry_max_) {
            RCLCPP_WARN(get_logger(), "Arm retry limit reached - back to INIT");
            setState(State::INIT);
            return;
        }
    }
}

void OffboardNode::handleTakeoff()
{
    px4_->publishOffboardControlMode(true, true, true);

    // Capture the takeoff origin (NED) on entry. Direct PX4 position control:
    // hold xy, climb to default_height.
    if (!have_takeoff_goal_) {
        const auto local_pos = px4_->getLocalPosition();
        if (!local_pos) {
            publishHold();
            return;
        }
        takeoff_target_x_ = local_pos->x;                          // NED
        takeoff_target_y_ = local_pos->y;
        takeoff_target_z_ = static_cast<float>(-default_height_);  // NED up = negative
        have_takeoff_goal_ = true;
        RCLCPP_INFO(get_logger(),
                    "Direct takeoff: climb to NED z=%.2f (height %.1f m)",
                    takeoff_target_z_, default_height_);
    }

    // Wait briefly for OFFBOARD mode / stream to engage before climbing.
    if (stateElapsedSec() < 0.3) {
        publishHold();
        return;
    }

    // Direct PX4 vertical climb: command the target altitude with an upward
    // velocity feed-forward (NED up = negative vz).
    const float vz = static_cast<float>(-takeoff_vel_);
    px4_->publishSetpoint(takeoff_target_x_, takeoff_target_y_, takeoff_target_z_,
                          0.0f, 0.0f, vz);

    // Takeoff complete once near the target altitude.
    const auto local_pos = px4_->getLocalPosition();
    if (local_pos && std::abs(local_pos->z - takeoff_target_z_) < 0.5) {
        RCLCPP_INFO(get_logger(), "Takeoff complete at z=%.2f m - entering IDLE",
                    takeoff_target_z_);
        // Set the IDLE hold to the takeoff altitude so the drone hovers there
        // instead of falling back to the default (0,0,0) hold.
        captureHold();
        have_takeoff_goal_ = false;
        planner_fail_count_ = 0;
        setState(State::IDLE);
    }
}

void OffboardNode::handleIdle()
{
    updatePlannerActivity();
    publishIdleHold();  // yaws toward the next goal while hovering

    // Planner failure while idle.
    if (super_->isPlannerFail()) {
        enterPlannerFail();
        return;
    }

    // Land command handled centrally in landCmdCallback.
    if (land_requested_) {
        land_requested_ = false;
        setState(State::LAND);
        return;
    }

    // Move to MOVE when there is a buffered goal and we are allowed to move.
    if (waypoints_->hasPendingGoal() && can_move_) {
        publishCurrentGoalIfChanged();
        RCLCPP_INFO(get_logger(), "New goal - entering MOVE");
        setState(State::MOVE);
    }
}

void OffboardNode::handleMove()
{
    updatePlannerActivity();

    // Forward the current buffered waypoint to SUPER when it has changed (e.g.
    // after the WaypointHandler advances to the next goal once one is reached).
    publishCurrentGoalIfChanged();

    // Land command interrupts navigation.
    if (land_requested_) {
        land_requested_ = false;
        setState(State::LAND);
        return;
    }

    // Enter PLANNER_FAIL only when the planner itself publishes the FAIL flag
    // (after `planner_fail_time` of continuous failure). A mere command-rate
    // drop / stall is handled below by holding, never by declaring a failure.
    if (super_->isPlannerFail()) {
        enterPlannerFail();
        return;
    }

    // Planner command rate drop. With no remaining goal this is a normal
    // completion -> IDLE. With a goal still pending, we do NOT declare a
    // failure immediately (that would bypass the planner's own 2 s fail flag);
    // instead we hold and wait for either the explicit FAIL flag (2 s) or the
    // longer stall timeout above to trigger PLANNER_FAIL.
    if (!planner_active_) {
        if (waypoints_->hasPendingGoal()) {
            publishHold();
            return;
        }
        RCLCPP_INFO(get_logger(), "Planner rate dropped - returning to IDLE");
        captureHold();
        // Goal reached / navigation finished: reset the planner so it
        // clears any residual FAIL state before the next goal.
        resetPlanner();
        setState(State::IDLE);
        return;
    }

    auto cmd = super_->getLatestCommand();
    if (!cmd) {
        publishHold();
        return;
    }

    // Forward the planner command (ENU -> NED).
    const auto &c = *cmd;
    float nx, ny, nz, vx, vy, vz, ax, ay, az;
    frame::enuToNed(c.position.x, c.position.y, c.position.z, nx, ny, nz);
    frame::enuToNed(c.velocity.x, c.velocity.y, c.velocity.z, vx, vy, vz);
    frame::enuToNed(c.acceleration.x, c.acceleration.y, c.acceleration.z, ax, ay, az);
    const float yaw_ned = static_cast<float>(frame::enuYawToNed(c.yaw));
    const float yawspeed_ned = static_cast<float>(frame::enuYawRateToNed(c.yaw_dot));
    px4_->publishSetpoint(nx, ny, nz, vx, vy, vz, yaw_ned, yawspeed_ned, ax, ay, az);

    // Goal reached -> back to IDLE (WaypointHandler advances the buffered
    // goals; when the buffer is empty this fires and we idle).
    if (!waypoints_->hasPendingGoal()) {
        RCLCPP_INFO(get_logger(), "Goal reached - entering IDLE");
        captureHold();
        planner_fail_count_ = 0;
        // Reset the planner to WAIT_GOAL so it clears any residual FAIL state
        // before the next goal is handed to it.
        resetPlanner();
        setState(State::IDLE);
    }
}

void OffboardNode::handlePlannerFail()
{
    px4_->publishOffboardControlMode(true, true, true);
    publishHold();

    // Cooldown between reset attempts. The reset request is asynchronous;
    // the response callback (see restartPlanner) resumes the FSM. We start
    // the cooldown when the request is sent so we do not re-send every tick.
    if ((now() - last_planner_reset_t_).seconds() < planner_reset_delay_) {
        return;
    }

    last_planner_reset_t_ = now();
    restartPlanner();
}

void OffboardNode::handleFailsafe()
{
    // Direct PX4 landing, no planner.
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
        planner_fail_count_ = 0;
        RCLCPP_INFO(get_logger(), "Failsafe landing complete - back to INIT");
        setState(State::INIT);
    }
}

void OffboardNode::handleEmergencyStop()
{
    publishHold();

    // Manual recovery: a fresh takeoff command (rising edge) brings us back.
    if (takeoff_requested_) {
        takeoff_requested_ = false;
        RCLCPP_INFO(get_logger(), "Manual recovery - back to INIT");
        setState(State::INIT);
    }
}

void OffboardNode::handleLand()
{
    px4_->publishOffboardControlMode(true, true, true);

    // Direct PX4 landing (no planner): hold the capture point xy and descend
    // to landing_z_ at landing_vel_.
    const float tx = have_hold_ ? hold_x_ : 0.0f;
    const float ty = have_hold_ ? hold_y_ : 0.0f;
    const float target_z = static_cast<float>(landing_z_);
    // NED descend = positive vz (down).
    px4_->publishSetpoint(tx, ty, target_z,
                          0.0f, 0.0f, static_cast<float>(landing_vel_));

    // Landed once near the ground (NED z >= landing_z_).
    const auto local_pos = px4_->getLocalPosition();
    if (local_pos && local_pos->z > target_z) {
        px4_->disarm();
        active_goal_ = geometry_msgs::msg::PoseStamped();
        have_hold_ = false;
        RCLCPP_INFO(get_logger(), "Direct landing complete - back to INIT");
        setState(State::INIT);
    }
}

}  // namespace offboard

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(offboard::OffboardNode)
