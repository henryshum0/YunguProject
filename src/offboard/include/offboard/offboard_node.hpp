#pragma once

#include <rclcpp/rclcpp.hpp>

#include <deque>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace offboard
{

/**
 * @brief PX4 offboard state machine.
 *
 *  State flow:
 *
 *    INIT ──► ARMING ──► SET_OFFBOARD ──► IDLE ◄──► PLANNER
 *                                          │
 *                                          ▼
 *                                       LANDING ──► LANDED
 *
 *  - IDLE:    no planner command (rate < planner_cmd_hz) → hover in place
 *  - PLANNER: planner commands arriving at >= planner_cmd_hz → forward them
 *             (ENU → NED conversion to PX4 TrajectorySetpoint)
 *
 *  The planner only publishes /planning/pos_cmd after it receives a goal,
 *  so this node detects "hand-over to planner" by measuring the command rate.
 */
class OffboardNode : public rclcpp::Node
{
public:
    explicit OffboardNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~OffboardNode() override = default;

private:
    enum class State {
        INIT,          ///< stream origin setpoints
        ARMING,        ///< send arm command, keep streaming
        SET_OFFBOARD,  ///< switch to OFFBOARD mode
        IDLE,          ///< hover in place, waiting for planner
        PLANNER,       ///< forward planner commands
        LANDING,       ///< descend and disarm
        LANDED,        ///< finished
    };

    // ------------------------------------------------------------------
    //  Configuration
    // ------------------------------------------------------------------
    double update_rate_{50.0};        ///< setpoint stream rate [Hz]
    double planner_cmd_hz_{10.0};     ///< cmd rate threshold for "planner active"
    double planner_enter_delay_{0.5}; ///< sustained active time before entering PLANNER [s]
    double planner_exit_delay_{1.0};  ///< sustained inactive time before leaving PLANNER [s]
    double arm_wait_{2.0};            ///< time in INIT before arming [s]
    double default_height_{1.5};      ///< NED hover height when no hold point exists [m, negative = up]
    double landing_vel_{0.5};         ///< descend speed [m/s]
    double landing_z_{0.15};          ///< NED z at which to disarm [m]
    double cmd_timeout_{0.5};         ///< max age of planner cmd before considered lost [s]
    std::string cmd_topic_{"/planning/pos_cmd"};
    /// PX4 local position topic. Note the "_v1" version suffix — this PX4 fork
    /// reports get_message_version()=1 for vehicle_local_position, so the
    /// MicroXRCEAgent advertises it with the versioned name.
    std::string local_pos_topic_{"/fmu/out/vehicle_local_position_v1"};
    /// PX4 vehicle status topic; MESSAGE_VERSION=4 → "_v4" suffix.
    std::string status_topic_{"/fmu/out/vehicle_status_v4"};

    // ------------------------------------------------------------------
    //  Publishers / Subscribers / Services
    // ------------------------------------------------------------------
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------------------
    //  Internal state
    // ------------------------------------------------------------------
    State state_{State::INIT};
    rclcpp::Time state_enter_t_;

    mars_quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_{nullptr};
    rclcpp::Time last_cmd_stamp_;
    std::deque<rclcpp::Time> cmd_stamps_;   ///< recent cmd stamps (1 s window)

    px4_msgs::msg::VehicleLocalPosition::SharedPtr local_pos_{nullptr};
    px4_msgs::msg::VehicleStatus::SharedPtr status_{nullptr};

    /// current NED hold position for IDLE / LANDING
    float hold_x_{0.0f};
    float hold_y_{0.0f};
    float hold_z_{0.0f};
    bool have_hold_{false};

    // planner activity (hysteresis)
    bool planner_active_{false};
    bool planner_cond_val_{false};
    rclcpp::Time planner_cond_t_;

    // ------------------------------------------------------------------
    //  Callbacks
    // ------------------------------------------------------------------
    void timerCallback();
    void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
    void localPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ------------------------------------------------------------------
    //  State helpers
    // ------------------------------------------------------------------
    void setState(State s);
    const char *stateName() const;
    static const char *stateNameOf(State s);
    double stateElapsedSec() const;

    // ------------------------------------------------------------------
    //  PX4 helpers
    // ------------------------------------------------------------------
    void publishOffboardMode(bool position, bool velocity, bool acceleration);
    void publishSetpoint(float x, float y, float z,
                         float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                         float yaw = NAN, float yawspeed = 0.0f,
                         float ax = NAN, float ay = NAN, float az = NAN);
    void publishHold();
    void sendCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void arm();
    void disarm();
    void setOffboardMode();

    // vehicle_status confirmation helpers
    bool isArmed() const;
    bool isOffboard() const;
    bool isDisarmed() const;

    // ------------------------------------------------------------------
    //  Planner rate measurement
    // ------------------------------------------------------------------
    void updatePlannerActivity();
    double cmdRateHz() const;

    // ------------------------------------------------------------------
    //  Conversion (ENU → NED)
    // ------------------------------------------------------------------
    static void enuToNedPos(double ex, double ey, double ez,
                            float &nx, float &ny, float &nz);
    static void enuToNedVel(double ex, double ey, double ez,
                            float &nx, float &ny, float &nz);
    static void enuToNedAcc(double ex, double ey, double ez,
                            float &nx, float &ny, float &nz);
};

}  // namespace offboard
