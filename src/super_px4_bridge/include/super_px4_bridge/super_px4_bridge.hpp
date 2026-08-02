#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>

namespace super_px4_bridge
{

class SuperPx4Bridge : public rclcpp::Node
{
public:
    explicit SuperPx4Bridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~SuperPx4Bridge() override = default;

private:
    // --- State machine ---
    enum class State {
        INIT,
        ARMING,
        SET_OFFBOARD,
        FOLLOW_SUPER,
        HOLD,
    };

    // --- Callbacks ---
    void timerCallback();
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

    // --- State helpers ---
    void setState(State new_state);
    double elapsedSec() const;

    // --- PX4 publishing ---
    void publishOffboardMode();
    void publishSetpoint(float x, float y, float z,
                         float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                         float yaw = NAN, float yawspeed = 0.0f);
    void sendCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void arm();
    void disarm();
    void setOffboardMode();

    // --- ENU → NED conversion ---
    static void enuToNedPosition(double enu_x, double enu_y, double enu_z,
                                 float &ned_x, float &ned_y, float &ned_z);
    static void enuToNedVelocity(double enu_vx, double enu_vy, double enu_vz,
                                 float &ned_vx, float &ned_vy, float &ned_vz);

    // --- Forwarding ---
    void forwardSuperCommand();

    // --- Parameters ---
    double update_rate_;
    double offboard_timeout_;
    std::string cmd_topic_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    // --- Subscriber ---
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- State ---
    State state_{State::INIT};
    rclcpp::Time state_enter_t_;
    mars_quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_{nullptr};
    rclcpp::Time last_cmd_stamp_;
};

}  // namespace super_px4_bridge
