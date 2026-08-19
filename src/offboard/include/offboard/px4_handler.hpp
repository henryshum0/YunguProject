#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace offboard
{

/**
 * @brief Owns all PX4-facing topic I/O for the offboard state machine.
 *
 * Wraps the publishers/subscribers that talk to the PX4 flight stack:
 *   - out: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
 *          /fmu/in/vehicle_command
 *   - in:  vehicle_local_position, vehicle_status
 *
 * The state machine (OffboardNode) drives the flow and calls the publish
 * helpers here; the latest local position / status messages can be read via
 * the getters.
 */
class Px4Handler
{
public:
    explicit Px4Handler(rclcpp::Node &node,
                        const std::string &local_pos_topic,
                        const std::string &status_topic);

    // --- Outgoing (state machine -> PX4) ---
    void publishOffboardControlMode(bool position, bool velocity, bool acceleration);
    void publishSetpoint(float x, float y, float z,
                         float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                         float yaw = NAN, float yawspeed = 0.0f,
                         float ax = NAN, float ay = NAN, float az = NAN);
    void sendCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void arm();
    void disarm();
    void setOffboardMode();

    // --- Incoming (PX4 -> state machine) ---
    std::shared_ptr<const px4_msgs::msg::VehicleLocalPosition> getLocalPosition() const;
    std::shared_ptr<const px4_msgs::msg::VehicleStatus> getStatus() const;
    bool isArmed() const;
    bool isOffboard() const;
    bool isDisarmed() const;

private:
    void localPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    rclcpp::Node &node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

    std::shared_ptr<px4_msgs::msg::VehicleLocalPosition> local_pos_{nullptr};
    std::shared_ptr<px4_msgs::msg::VehicleStatus> status_{nullptr};
};

}  // namespace offboard
