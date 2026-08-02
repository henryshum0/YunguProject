#include "super_px4_bridge/super_px4_bridge.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace super_px4_bridge
{

SuperPx4Bridge::SuperPx4Bridge(const rclcpp::NodeOptions &options)
    : Node("super_px4_bridge", options)
{
    // --- Parameters ---
    update_rate_ = declare_parameter("update_rate", 50.0);
    offboard_timeout_ = declare_parameter("offboard_timeout", 5.0);
    cmd_topic_ = declare_parameter("cmd_topic", "/planning/pos_cmd");

    const auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));

    // --- QoS ---
    auto qos_best_effort = rclcpp::QoS(10)
        .best_effort()
        .transient_local();

    auto qos_reliable = rclcpp::QoS(10)
        .reliable()
        .keep_last(10);

    // --- Publishers (to PX4) ---
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_best_effort);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_best_effort);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_best_effort);

    // --- Subscriber (from SUPER) ---
    cmd_sub_ = create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        cmd_topic_, qos_reliable,
        std::bind(&SuperPx4Bridge::cmdCallback, this, std::placeholders::_1));

    // --- Timer ---
    timer_ = create_wall_timer(update_period,
                               std::bind(&SuperPx4Bridge::timerCallback, this));

    state_enter_t_ = now();

    RCLCPP_INFO(get_logger(), "SuperPx4Bridge started, listening on %s", cmd_topic_.c_str());
}

// ======================================================================
//  State helpers
// ======================================================================

void SuperPx4Bridge::setState(State new_state)
{
    RCLCPP_INFO(get_logger(), "State: %d → %d", static_cast<int>(state_), static_cast<int>(new_state));
    state_ = new_state;
    state_enter_t_ = now();
}

double SuperPx4Bridge::elapsedSec() const
{
    return (now() - state_enter_t_).seconds();
}

// ======================================================================
//  SUPER callback
// ======================================================================

void SuperPx4Bridge::cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    latest_cmd_ = msg;
    last_cmd_stamp_ = now();
}

// ======================================================================
//  PX4 publishing
// ======================================================================

void SuperPx4Bridge::publishOffboardMode()
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

void SuperPx4Bridge::publishSetpoint(float x, float y, float z,
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

void SuperPx4Bridge::sendCommand(uint16_t command, float param1, float param2)
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

void SuperPx4Bridge::arm()
{
    RCLCPP_INFO(get_logger(), "Sending ARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void SuperPx4Bridge::disarm()
{
    RCLCPP_INFO(get_logger(), "Sending DISARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void SuperPx4Bridge::setOffboardMode()
{
    RCLCPP_INFO(get_logger(), "Requesting OFFBOARD mode");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

// ======================================================================
//  ENU → NED conversion
// ======================================================================

void SuperPx4Bridge::enuToNedPosition(double enu_x, double enu_y, double enu_z,
                                      float &ned_x, float &ned_y, float &ned_z)
{
    ned_x = static_cast<float>(enu_y);
    ned_y = static_cast<float>(enu_x);
    ned_z = static_cast<float>(-enu_z);
}

void SuperPx4Bridge::enuToNedVelocity(double enu_vx, double enu_vy, double enu_vz,
                                      float &ned_vx, float &ned_vy, float &ned_vz)
{
    ned_vx = static_cast<float>(enu_vy);
    ned_vy = static_cast<float>(enu_vx);
    ned_vz = static_cast<float>(-enu_vz);
}

// ======================================================================
//  Timer callback
// ======================================================================

void SuperPx4Bridge::timerCallback()
{
    publishOffboardMode();

    switch (state_) {
        case State::INIT: {
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (elapsedSec() > 2.0) {
                setState(State::ARMING);
            }
            break;
        }
        case State::ARMING: {
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (elapsedSec() < 0.1) {
                arm();
            }
            if (elapsedSec() > 3.0) {
                setState(State::SET_OFFBOARD);
            }
            break;
        }
        case State::SET_OFFBOARD: {
            publishSetpoint(0.0f, 0.0f, 0.0f);
            if (elapsedSec() < 0.1) {
                setOffboardMode();
            }
            if (elapsedSec() > 3.0) {
                setState(State::FOLLOW_SUPER);
            }
            break;
        }
        case State::FOLLOW_SUPER: {
            forwardSuperCommand();
            break;
        }
        case State::HOLD: {
            if (latest_cmd_) {
                const auto &p = latest_cmd_->position;
                float nx, ny, nz;
                enuToNedPosition(p.x, p.y, p.z, nx, ny, nz);
                publishSetpoint(nx, ny, nz);
            }
            // Auto-recover if SUPER commands resume
            if (latest_cmd_) {
                double elapsed = (now() - last_cmd_stamp_).seconds();
                if (elapsed < 0.5) {
                    RCLCPP_INFO(get_logger(), "SUPER commands recovered");
                    setState(State::FOLLOW_SUPER);
                }
            }
            break;
        }
    }
}

void SuperPx4Bridge::forwardSuperCommand()
{
    if (!latest_cmd_) {
        publishSetpoint(0.0f, 0.0f, 0.0f);
        return;
    }

    // Check timeout
    double elapsed = (now() - last_cmd_stamp_).seconds();
    if (elapsed > offboard_timeout_) {
        RCLCPP_WARN(get_logger(),
                    "SUPER command timeout (%.1fs > %.1fs), holding position",
                    elapsed, offboard_timeout_);
        setState(State::HOLD);
        return;
    }

    const auto &cmd = *latest_cmd_;

    // Convert ENU → NED
    float ned_x, ned_y, ned_z;
    float ned_vx, ned_vy, ned_vz;
    enuToNedPosition(cmd.position.x, cmd.position.y, cmd.position.z,
                     ned_x, ned_y, ned_z);
    enuToNedVelocity(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                     ned_vx, ned_vy, ned_vz);

    // Yaw: SUPER yaw in ENU (0=East, CCW+) → PX4 yaw in NED (0=North, CW+)
    constexpr float M_PI_2 = 1.57079632679f;
    float yaw_ned = M_PI_2 - static_cast<float>(cmd.yaw);
    float yawspeed_ned = -static_cast<float>(cmd.yaw_dot);

    publishSetpoint(ned_x, ned_y, ned_z,
                    ned_vx, ned_vy, ned_vz,
                    yaw_ned, yawspeed_ned);
}

}  // namespace super_px4_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(super_px4_bridge::SuperPx4Bridge)
