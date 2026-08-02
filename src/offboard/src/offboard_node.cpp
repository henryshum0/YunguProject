#include "offboard/offboard_node.hpp"

#include <algorithm>
#include <chrono>

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
    offboard_wait_ = declare_parameter("offboard_wait", offboard_wait_);
    takeoff_height_ = declare_parameter("takeoff_height", takeoff_height_);
    takeoff_vel_ = declare_parameter("takeoff_vel", takeoff_vel_);
    landing_vel_ = declare_parameter("landing_vel", landing_vel_);
    landing_z_ = declare_parameter("landing_z", landing_z_);
    cmd_timeout_ = declare_parameter("cmd_timeout", cmd_timeout_);
    cmd_topic_ = declare_parameter("cmd_topic", cmd_topic_);
    local_pos_topic_ = declare_parameter("local_pos_topic", local_pos_topic_);
    goal_topic_ = declare_parameter("goal_topic", goal_topic_);
    goal_marker_topic_ = declare_parameter("goal_marker_topic", goal_marker_topic_);

    const auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));

    // --- QoS ---
    auto qos_px4 = rclcpp::QoS(10)
        .best_effort()
        .transient_local();
    // SUPER publishes best_effort/volatile/keep_last(1) — must match exactly
    auto qos_super = rclcpp::QoS(1)
        .best_effort()
        .keep_last(1)
        .durability_volatile();

    // --- Publishers (to PX4) ---
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_px4);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_px4);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_px4);
    // Transient-local so a late-starting RViz still sees the latest goal marker.
    auto qos_marker = rclcpp::QoS(1).transient_local();
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        goal_marker_topic_, qos_marker);

    // --- Subscribers ---
    cmd_sub_ = create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        cmd_topic_, qos_super,
        std::bind(&OffboardNode::cmdCallback, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_pos_topic_, qos_px4,
        std::bind(&OffboardNode::localPosCallback, this, std::placeholders::_1));
    // RViz "2D Goal Pose" publishes reliable/volatile; accept it.
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, qos_super,
        std::bind(&OffboardNode::goalCallback, this, std::placeholders::_1));

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

void OffboardNode::cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    latest_cmd_ = msg;
    last_cmd_stamp_ = now();
    cmd_stamps_.push_back(now());
}

void OffboardNode::localPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    local_pos_ = msg;
}

void OffboardNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Goal received: (%.2f, %.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    publishGoalMarker(*msg);
}

void OffboardNode::publishGoalMarker(const geometry_msgs::msg::PoseStamped &goal)
{
    // Green sphere at the clicked goal position.
    visualization_msgs::msg::Marker sphere;
    sphere.header = goal.header;
    sphere.ns = "goal";
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = goal.pose;
    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;
    sphere.color.r = 0.0f;
    sphere.color.g = 1.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;
    goal_marker_pub_->publish(sphere);

    // Cyan arrow showing the goal yaw orientation.
    visualization_msgs::msg::Marker arrow;
    arrow.header = goal.header;
    arrow.ns = "goal";
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = goal.pose;
    arrow.scale.x = 0.6;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.12;
    arrow.color.r = 0.0f;
    arrow.color.g = 1.0f;
    arrow.color.b = 1.0f;
    arrow.color.a = 1.0f;
    goal_marker_pub_->publish(arrow);
}

void OffboardNode::landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (state_ == State::LANDING || state_ == State::LANDED) {
        res->success = false;
        res->message = "Already landing/landed";
        return;
    }
    // Remember the current hold point (or last cmd) so landing keeps xy
    if (local_pos_) {
        hold_x_ = local_pos_->x;
        hold_y_ = local_pos_->y;
        have_hold_ = true;
    }
    RCLCPP_INFO(get_logger(), "Landing requested");
    setState(State::LANDING);
    res->success = true;
    res->message = "Landing";
}

// ======================================================================
//  PX4 helpers
// ======================================================================

void OffboardNode::publishOffboardMode()
{
    px4_msgs::msg::OffboardControlMode m;
    m.position = true;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.thrust_and_torque = false;
    m.direct_actuator = false;
    m.timestamp = now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(m);
}

void OffboardNode::publishSetpoint(float x, float y, float z,
                                   float vx, float vy, float vz,
                                   float yaw, float yawspeed)
{
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.position = {x, y, z};
    sp.velocity = {vx, vy, vz};
    sp.acceleration = {NAN, NAN, NAN};
    sp.jerk = {NAN, NAN, NAN};
    sp.yaw = yaw;
    sp.yawspeed = yawspeed;
    trajectory_pub_->publish(sp);
}

void OffboardNode::publishHold()
{
    publishSetpoint(hold_x_, hold_y_, hold_z_, 0.0f, 0.0f, 0.0f);
}

void OffboardNode::sendCommand(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

void OffboardNode::arm()
{
    RCLCPP_INFO(get_logger(), "Sending ARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardNode::disarm()
{
    RCLCPP_INFO(get_logger(), "Sending DISARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void OffboardNode::setOffboardMode()
{
    RCLCPP_INFO(get_logger(), "Requesting OFFBOARD mode");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

// ======================================================================
//  Planner rate measurement (hysteresis)
// ======================================================================

double OffboardNode::cmdRateHz() const
{
    // Remove stamps older than 1 s, then count what's left.
    const rclcpp::Time cutoff = now() - rclcpp::Duration(1, 0);
    // deque is const here; count without mutating
    size_t n = 0;
    for (auto it = cmd_stamps_.rbegin(); it != cmd_stamps_.rend(); ++it) {
        if (*it < cutoff) {
            break;
        }
        n++;
    }
    return static_cast<double>(n);
}

void OffboardNode::updatePlannerActivity()
{
    // Prune stale stamps once in a while (cheap enough every tick)
    const rclcpp::Time cutoff = now() - rclcpp::Duration(1, 0);
    while (!cmd_stamps_.empty() && cmd_stamps_.front() < cutoff) {
        cmd_stamps_.pop_front();
    }

    const bool cond = cmdRateHz() >= planner_cmd_hz_;
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
//  ENU → NED conversion
// ======================================================================

void OffboardNode::enuToNedPos(double ex, double ey, double ez,
                               float &nx, float &ny, float &nz)
{
    nx = static_cast<float>(ey);
    ny = static_cast<float>(ex);
    nz = static_cast<float>(-ez);
}

void OffboardNode::enuToNedVel(double ex, double ey, double ez,
                               float &nx, float &ny, float &nz)
{
    nx = static_cast<float>(ey);
    ny = static_cast<float>(ex);
    nz = static_cast<float>(-ez);
}

// ======================================================================
//  Main state machine
// ======================================================================

void OffboardNode::timerCallback()
{
    publishOffboardMode();

    switch (state_) {
        // --------------------------------------------------------------
        case State::INIT: {
            // Stream origin so PX4 sees the offboard stream before arming.
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (stateElapsedSec() > arm_wait_) {
                setState(State::ARMING);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::ARMING: {
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (stateElapsedSec() < 0.1) {
                arm();
            }
            if (stateElapsedSec() > offboard_wait_) {
                setState(State::SET_OFFBOARD);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::SET_OFFBOARD: {
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (stateElapsedSec() < 0.1) {
                setOffboardMode();
            }
            if (stateElapsedSec() > 3.0) {
                // Take off above the start point.
                hold_x_ = 0.0f;
                hold_y_ = 0.0f;
                have_hold_ = true;
                setState(State::TAKEOFF);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::TAKEOFF: {
            const float target_z = static_cast<float>(-takeoff_height_);
            // Climb with a fixed velocity; stop when close to target.
            publishSetpoint(hold_x_, hold_y_, target_z,
                            0.0f, 0.0f, static_cast<float>(takeoff_vel_));

            const bool reached = local_pos_ &&
                std::abs(local_pos_->z - target_z) < 0.1;
            if (reached || stateElapsedSec() > 15.0) {
                hold_z_ = target_z;
                RCLCPP_INFO(get_logger(), "Takeoff complete (z = %.2f m)", hold_z_);
                setState(State::IDLE);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::IDLE: {
            updatePlannerActivity();
            publishHold();

            if (planner_active_) {
                // Remember the current hold so we can return to it later.
                if (local_pos_) {
                    hold_x_ = local_pos_->x;
                    hold_y_ = local_pos_->y;
                    hold_z_ = local_pos_->z;
                }
                setState(State::PLANNER);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::PLANNER: {
            updatePlannerActivity();
            if (!planner_active_) {
                // Planner stopped → freeze at last commanded position.
                if (latest_cmd_) {
                    enuToNedPos(latest_cmd_->position.x,
                                latest_cmd_->position.y,
                                latest_cmd_->position.z,
                                hold_x_, hold_y_, hold_z_);
                    have_hold_ = true;
                }
                setState(State::IDLE);
                break;
            }

            if (!latest_cmd_) {
                publishHold();
                break;
            }

            // Forward the planner command (ENU → NED).
            const auto &cmd = *latest_cmd_;
            float nx, ny, nz, vx, vy, vz;
            enuToNedPos(cmd.position.x, cmd.position.y, cmd.position.z, nx, ny, nz);
            enuToNedVel(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z, vx, vy, vz);

            // SUPER yaw is ENU (0 = East, CCW+); PX4 is NED (0 = North, CW+).
            constexpr float PI_HALF = 1.57079632679f;
            const float yaw_ned = PI_HALF - static_cast<float>(cmd.yaw);
            const float yawspeed_ned = -static_cast<float>(cmd.yaw_dot);

            publishSetpoint(nx, ny, nz, vx, vy, vz, yaw_ned, yawspeed_ned);
            break;
        }
        // --------------------------------------------------------------
        case State::LANDING: {
            const float target_z = static_cast<float>(landing_z_);
            if (have_hold_) {
                publishSetpoint(hold_x_, hold_y_, target_z,
                                0.0f, 0.0f, static_cast<float>(-landing_vel_));
            } else {
                publishSetpoint(0.0f, 0.0f, target_z,
                                0.0f, 0.0f, static_cast<float>(-landing_vel_));
            }

            const bool on_ground = local_pos_ && local_pos_->z > target_z;
            if (on_ground || stateElapsedSec() > 20.0) {
                disarm();
                setState(State::LANDED);
            }
            break;
        }
        // --------------------------------------------------------------
        case State::LANDED: {
            // Stop streaming setpoints; PX4 drops out of offboard.
            break;
        }
    }
}

}  // namespace offboard

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(offboard::OffboardNode)
