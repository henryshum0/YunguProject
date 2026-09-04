#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include "offboard_fsm/srv/clear_waypoints.hpp"
#include "offboard_fsm/srv/queue_waypoints.hpp"
#include "offboard/px4_handler.hpp"
#include "offboard/super_handler.hpp"
#include "offboard/waypoint_handler.hpp"
#include "offboard/frame_conversion.hpp"

namespace offboard
{

/** PX4 offboard state machine for planner-driven flight. */
class OffboardNode : public rclcpp::Node
{
public:
    explicit OffboardNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~OffboardNode() override = default;

private:
    enum class State {
        INIT,          ///< wait for system readiness + takeoff command
        ARMING,        ///< try to arm, retry up to N times
        TAKEOFF,       ///< planner-driven ascent to the takeoff height
        IDLE,          ///< hover, waiting for a goal
        MOVE,          ///< forward planner commands (navigation)
        LAND,          ///< planner-driven landing
    };

    double update_rate_{50.0};        ///< setpoint stream rate [Hz]
    double planner_cmd_hz_{10.0};     ///< cmd rate threshold for "planner active"
    double arm_retry_delay_{5.0};     ///< delay between arm attempts [s]
    int arm_retry_max_{3};            ///< max arm attempts before back to INIT
    double default_height_{1.5};      ///< NED takeoff hover height (negative = up) [m]
    double takeoff_vel_{0.5};         ///< climb speed [m/s] (direct PX4 takeoff)
    double landing_vel_{0.5};         ///< descend speed [m/s] (FAILSAFE direct land)
    double landing_z_{0.15};          ///< NED z at which to disarm [m]
    double yaw_align_thresh_{0.35};   ///< rad, heading-to-goal tolerance (heading_ok)
    double waypoint_reached_dist_{3.0}; ///< horizontal distance to consider a waypoint reached
    double planner_reset_delay_{5.0};   ///< delay between planner reset attempts [s]

    std::string cmd_topic_{"/planning/pos_cmd"};
    std::string local_pos_topic_{"/fmu/out/vehicle_local_position_v1"};
    std::string status_topic_{"/fmu/out/vehicle_status_v4"};
    std::string land_detected_topic_{"/fmu/out/vehicle_land_detected"};
    std::string goal_topic_{"/goal_pose"};
    std::string planner_state_topic_{"fsm/planner_state"};
    std::string goal_status_topic_{"fsm/goal_status"};
    std::string lio_state_topic_{"fastlio/lio_state"};
    std::string planner_reset_service_{"/fsm_node/reset"};
    std::string takeoff_cmd_topic_{"/takeoff_cmd"};
    std::string land_cmd_topic_{"/land_cmd"};
    std::string waypoint_queue_service_{"/waypoint_buffer"};
    std::string clear_waypoints_service_{"/waypoint_buffer/clear"};

    std::unique_ptr<Px4Handler> px4_;
    std::unique_ptr<SuperHandler> super_;
    std::unique_ptr<WaypointHandler> waypoints_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Service<offboard_fsm::srv::QueueWaypoints>::SharedPtr waypoint_queue_srv_;
    rclcpp::Service<offboard_fsm::srv::ClearWaypoints>::SharedPtr clear_waypoints_srv_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_{State::INIT};
    rclcpp::Time state_enter_t_;

    /// current NED hold position for IDLE / landing
    float hold_x_{0.0f};
    float hold_y_{0.0f};
    float hold_z_{0.0f};
    bool have_hold_{false};

    int arm_retry_count_{0};
    rclcpp::Time last_arm_t_;
    bool planner_reset_in_flight_{false};
    rclcpp::Time last_planner_reset_t_;

    float takeoff_target_x_{0.0f};
    float takeoff_target_y_{0.0f};
    float takeoff_target_z_{0.0f};
    bool have_takeoff_goal_{false};
    geometry_msgs::msg::PoseStamped active_goal_;

    bool takeoff_requested_{false};
    bool land_requested_{false};

    bool planner_active_{false};
    bool planner_cond_val_{false};
    rclcpp::Time planner_cond_t_;
    bool stuck_recovery_attempted_{false};

    void timerCallback();
    void landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void queueWaypointsCallback(
        const std::shared_ptr<offboard_fsm::srv::QueueWaypoints::Request> req,
        std::shared_ptr<offboard_fsm::srv::QueueWaypoints::Response> res);
    void clearWaypointsCallback(
        const std::shared_ptr<offboard_fsm::srv::ClearWaypoints::Request> req,
        std::shared_ptr<offboard_fsm::srv::ClearWaypoints::Response> res);
    void takeoffCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void landCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void setState(State s);
    const char *stateName() const;
    static const char *stateNameOf(State s);
    double stateElapsedSec() const;

    void handleInit();
    void handleArming();
    void handleTakeoff();
    void handleIdle();
    void handleMove();
    void handleLand();

    void publishHold();
    void publishIdleHold();
    void captureHold();
    void publishGoalToPlanner(const geometry_msgs::msg::PoseStamped &goal);
    bool publishCurrentGoal();

    bool systemReady() const;
    std::optional<geometry_msgs::msg::PoseStamped> headingTarget() const;
    bool headingOk() const;
    void updatePlannerActivity();

    void restartPlanner();
};

}  // namespace offboard
