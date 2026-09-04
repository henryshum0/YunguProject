#include "offboard/px4_handler.hpp"

#include <cmath>

namespace offboard
{

Px4Handler::Px4Handler(rclcpp::Node &node,
                       const std::string &local_pos_topic,
                       const std::string &status_topic,
                       const std::string &land_detected_topic)
    : node_(node)
{
    auto qos_px4 = rclcpp::QoS(10)
        .best_effort()
        .transient_local();

    // --- Publishers (to PX4) ---
    offboard_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_px4);
    trajectory_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_px4);
    cmd_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_px4);

    // --- Subscribers (from PX4) ---
    local_pos_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_pos_topic, qos_px4,
        std::bind(&Px4Handler::localPosCallback, this, std::placeholders::_1));
    status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, qos_px4,
        std::bind(&Px4Handler::statusCallback, this, std::placeholders::_1));
    land_detected_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLandDetected>(
        land_detected_topic, qos_px4,
        std::bind(&Px4Handler::landDetectedCallback, this, std::placeholders::_1));
}

// ======================================================================
//  Outgoing (state machine -> PX4)
// ======================================================================

void Px4Handler::publishOffboardControlMode(bool position, bool velocity,
                                            bool acceleration)
{
    px4_msgs::msg::OffboardControlMode m;
    m.position = position;
    m.velocity = velocity;
    m.acceleration = acceleration;
    m.attitude = false;
    m.body_rate = false;
    m.thrust_and_torque = false;
    m.direct_actuator = false;
    m.timestamp = node_.now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(m);
}

void Px4Handler::publishSetpoint(float x, float y, float z,
                                 float vx, float vy, float vz,
                                 float yaw, float yawspeed,
                                 float ax, float ay, float az)
{
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.position = {x, y, z};
    sp.velocity = {vx, vy, vz};
    // Provide the trajectory's acceleration (NED) as feedforward; defaults to
    // NaN (not controlled) for hover/takeoff/landing setpoints.
    sp.acceleration = {ax, ay, az};
    sp.jerk = {NAN, NAN, NAN};
    sp.yaw = yaw;
    sp.yawspeed = yawspeed;
    trajectory_pub_->publish(sp);
}

void Px4Handler::sendCommand(uint16_t command, float param1, float param2)
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
    msg.timestamp = node_.now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

void Px4Handler::arm()
{
    RCLCPP_INFO(node_.get_logger(), "Sending ARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void Px4Handler::disarm()
{
    RCLCPP_INFO(node_.get_logger(), "Sending DISARM command");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void Px4Handler::setOffboardMode()
{
    RCLCPP_INFO(node_.get_logger(), "Requesting OFFBOARD mode");
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

// ======================================================================
//  Incoming (PX4 -> state machine)
// ======================================================================

void Px4Handler::localPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    local_pos_ = msg;
}

void Px4Handler::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    status_ = msg;
}

void Px4Handler::landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    land_detected_ = msg;
}

std::shared_ptr<const px4_msgs::msg::VehicleLocalPosition>
Px4Handler::getLocalPosition() const
{
    return local_pos_;
}

std::shared_ptr<const px4_msgs::msg::VehicleStatus>
Px4Handler::getStatus() const
{
    return status_;
}

bool Px4Handler::isArmed() const
{
    return status_ != nullptr &&
           status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool Px4Handler::isOffboard() const
{
    return status_ != nullptr &&
           status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool Px4Handler::isDisarmed() const
{
    return status_ != nullptr &&
           status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

bool Px4Handler::isLanded() const
{
    // Prefer the dedicated PX4 landing detector once it has reported.
    if (land_detected_) {
        return land_detected_->landed;
    }
    // Fallback: close to the ground according to the range sensor / estimate.
    if (local_pos_ && local_pos_->dist_bottom_valid) {
        return local_pos_->dist_bottom < 0.3f;
    }
    return false;
}

bool Px4Handler::isInAir() const
{
    return !isLanded() && hasValidPosition();
}

uint8_t Px4Handler::navState() const
{
    return status_ ? status_->nav_state
                   : px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;
}

bool Px4Handler::hasValidPosition() const
{
    if (!local_pos_ || !local_pos_->xy_valid || !local_pos_->z_valid) {
        return false;
    }
    return std::isfinite(local_pos_->x) && std::isfinite(local_pos_->y) &&
           std::isfinite(local_pos_->z);
}

}  // namespace offboard
