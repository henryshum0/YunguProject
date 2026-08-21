#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include "offboard/px4_handler.hpp"
#include "offboard/super_handler.hpp"
#include "offboard/waypoint_handler.hpp"

namespace offboard
{

/**
 * @brief PX4 offboard state machine (planner-driven).
 *
 *  State flow (planner-driven flight):
 *
 *    INIT ─► ARMING ─► TAKEOFF ─► IDLE ◄──► MOVE
 *      ▲       │        │  ▲        │         │
 *      │       │        │  └────────┴──► PLANNER_FAIL ─► FAILSAFE
 *      │       │        │
 *      │       │        └────► (planner fail / land cmd / emergency)
 *      │       │
 *      │       ▼
 *      └─ EMERGENCY_STOP ◄── (critical failure from any state)
 *
 *  - INIT:      wait until odometry, FAST-LIO and the planner are ready and the
 *               vehicle is landed & disarmed; arm OFFBOARD mode; wait for a
 *               takeoff command (/takeoff_cmd). Critical failures -> EMERGENCY_STOP.
 *  - ARMING:    try to arm; retry up to 3 times with a 5 s delay; fall back to
 *               INIT if exhausted. Heartbeat loss -> INIT.
 *  - TAKEOFF:   hand the takeoff goal to the planner (no more manual quintic
 *               trajectory). Planner fail -> PLANNER_FAIL. Reached altitude -> IDLE.
 *  - IDLE:      hover, waiting for goals. With a buffered goal and can_move_,
 *               publish it to the planner and go to MOVE.
 *  - MOVE:      forward planner commands (ENU -> NED) to PX4. Planner fail ->
 *               PLANNER_FAIL; planner rate drop -> IDLE; goal reached -> IDLE;
 *               land command -> LAND.
 *  - PLANNER_FAIL: restart the planner via its reset service, re-publish the
 *               current goal; after 3 consecutive failures -> FAILSAFE.
 *  - FAILSAFE:  land with direct PX4 commands (no planner), then back to INIT.
 *  - LAND:      planner-driven landing (goal directly below the drone), then
 *               disarm and back to INIT.
 *  - EMERGENCY_STOP: hold position; wait for manual recovery -> INIT.
 *
 *  Topic I/O and per-subdomain logic are delegated to Px4Handler (PX4 stack),
 *  SuperHandler (SUPER planner + FAST-LIO state) and WaypointHandler (buffered
 *  waypoint following). Goal marking lives in the goal_marker_node. Frame
 *  conversions live in offboard::frame (frame_conversion.hpp).
 */
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
        PLANNER_FAIL,  ///< restart planner, then resume; N fails -> FAILSAFE
        FAILSAFE,      ///< land with direct PX4 commands (no planner)
        EMERGENCY_STOP,///< hold, wait for manual recovery
        LAND,          ///< planner-driven landing
    };

    // ------------------------------------------------------------------
    //  Configuration
    // ------------------------------------------------------------------
    double update_rate_{50.0};        ///< setpoint stream rate [Hz]
    double planner_cmd_hz_{10.0};     ///< cmd rate threshold for "planner active"
    double arm_retry_delay_{5.0};     ///< delay between arm attempts [s]
    int arm_retry_max_{3};            ///< max arm attempts before back to INIT
    int planner_fail_retry_max_{3};   ///< max planner restarts before FAILSAFE
    double default_height_{1.5};      ///< NED takeoff hover height (negative = up) [m]
    double takeoff_vel_{0.5};         ///< climb speed [m/s] (direct PX4 takeoff)
    double landing_vel_{0.5};         ///< descend speed [m/s] (FAILSAFE direct land)
    double landing_z_{0.15};          ///< NED z at which to disarm [m]
    double yaw_align_thresh_{0.35};   ///< rad, heading-to-goal for can_move_
    double waypoint_reached_dist_{3.0}; ///< horizontal distance to consider a waypoint reached
    double planner_reset_delay_{5.0};   ///< delay between planner reset attempts [s]
    double planner_stall_timeout_{5.0}; ///< time without planner commands before treated as a planner failure [s]

    std::string cmd_topic_{"/planning/pos_cmd"};
    /// PX4 local position topic. Note the "_v1" version suffix — this PX4 fork
    /// reports get_message_version()=1 for vehicle_local_position, so the
    /// MicroXRCEAgent advertises it with the versioned name.
    std::string local_pos_topic_{"/fmu/out/vehicle_local_position_v1"};
    /// PX4 vehicle status topic; MESSAGE_VERSION=4 → "_v4" suffix.
    std::string status_topic_{"/fmu/out/vehicle_status_v4"};
    /// Topic the current waypoint is published to for SUPER (one at a time).
    std::string goal_topic_{"/goal_pose"};
    /// Planner FSM state topic (published by super_planner's fsm_node).
    std::string planner_state_topic_{"fsm/planner_state"};
    /// FAST-LIO odometry health topic (published by fastlio_mapping).
    std::string lio_state_topic_{"fastlio/lio_state"};
    /// Planner reset service (super_planner's ~/reset).
    std::string planner_reset_service_{"/fsm_node/reset"};
    /// User takeoff command topic (std_msgs/Bool; true = take off).
    std::string takeoff_cmd_topic_{"/takeoff_cmd"};
    /// User land command topic (std_msgs/Bool; true = land).
    std::string land_cmd_topic_{"/land_cmd"};

    // ------------------------------------------------------------------
    //  Topic I/O handlers
    // ------------------------------------------------------------------
    std::unique_ptr<Px4Handler> px4_;
    std::unique_ptr<SuperHandler> super_;
    std::unique_ptr<WaypointHandler> waypoints_;

    // ------------------------------------------------------------------
    //  Publishers / Subscribers / Services (node-local, not PX4/SUPER)
    // ------------------------------------------------------------------
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------------------
    //  Internal state
    // ------------------------------------------------------------------
    State state_{State::INIT};
    rclcpp::Time state_enter_t_;

    /// current NED hold position for IDLE / landing
    float hold_x_{0.0f};
    float hold_y_{0.0f};
    float hold_z_{0.0f};
    bool have_hold_{false};

    // --- Arm / planner failure retry bookkeeping ---
    int arm_retry_count_{0};
    rclcpp::Time last_arm_t_;
    int planner_fail_count_{0};
    /// State to resume after a planner reset (TAKEOFF or MOVE).
    State fail_resume_state_{State::TAKEOFF};
    /// Timestamp of the last planner reset attempt (cooldown).
    rclcpp::Time last_planner_reset_t_;

    // --- Takeoff / navigation goal targets (ENU, planner frame) ---
    float takeoff_target_x_{0.0f};
    float takeoff_target_y_{0.0f};
    float takeoff_target_z_{0.0f};
    bool have_takeoff_goal_{false};
    /// The goal currently handed to the planner (ENU), for re-publish on reset.
    geometry_msgs::msg::PoseStamped active_goal_;
    /// Signature (position + yaw) of the goal most recently published to SUPER,
    /// so MOVE can detect when the current buffered goal changes and re-publish.
    bool goal_sent_{false};
    double sent_goal_x_{0.0}, sent_goal_y_{0.0}, sent_goal_z_{0.0}, sent_goal_yaw_{0.0};

    // --- Movement gate ---
    bool can_move_{false};

    // --- Command flags from user topics ---
    bool takeoff_requested_{false};
    bool land_requested_{false};

    // --- planner activity (hysteresis) ---
    bool planner_active_{false};
    bool planner_cond_val_{false};
    rclcpp::Time planner_cond_t_;
    /// Time the last planner command was received (stall detection).
    rclcpp::Time last_cmd_time_;

    // ------------------------------------------------------------------
    //  Callbacks
    // ------------------------------------------------------------------
    void timerCallback();
    void landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void takeoffCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void landCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // ------------------------------------------------------------------
    //  State helpers
    // ------------------------------------------------------------------
    void setState(State s);
    const char *stateName() const;
    static const char *stateNameOf(State s);
    double stateElapsedSec() const;

    // ------------------------------------------------------------------
    //  Per-state handlers (called from timerCallback's dispatch)
    // ------------------------------------------------------------------
    void handleInit();
    void handleArming();
    void handleTakeoff();
    void handleIdle();
    void handleMove();
    void handlePlannerFail();
    void handleFailsafe();
    void handleEmergencyStop();
    void handleLand();

    // ------------------------------------------------------------------
    //  Setpoint / goal helpers
    // ------------------------------------------------------------------
    void publishHold();
    /// Like publishHold() but yaws the drone toward the current buffered goal
    /// (used in IDLE so the heading points at the next waypoint).
    void publishIdleHold();
    /// Snapshot the current local position into the IDLE hold so the drone
    /// hovers at its present spot (avoiding a fall to the default 0,0,0 hold).
    void captureHold();
    void publishGoalToPlanner(const geometry_msgs::msg::PoseStamped &goal);
    /// Forward the current buffered waypoint to SUPER (/goal_pose) when it has
    /// changed since the last send. Returns true when a new goal was sent.
    bool publishCurrentGoalIfChanged();

    // ------------------------------------------------------------------
    //  System readiness / conditions
    // ------------------------------------------------------------------
    bool systemReady() const;
    /// Compute the can_move_ gate (reached current goal and yaw towards next).
    bool computeCanMove() const;
    void updatePlannerActivity();
    /// True when the planner has stopped producing commands for
    /// planner_stall_timeout_ while a goal is still pending (stalled).
    bool plannerStalled() const;

    // ------------------------------------------------------------------
    //  Planner failure handling
    // ------------------------------------------------------------------
    void enterPlannerFail();
    /// Asynchronously restart the planner (reset service). The response
    /// callback re-publishes the active goal and resumes fail_resume_state_.
    void restartPlanner();
    /// Asynchronously reset the planner to WAIT_GOAL (reset service) without
    /// re-publishing a goal or resuming a state. Used when a goal is reached,
    /// so the planner clears any residual FAIL state before the next goal.
    void resetPlanner();
};

}  // namespace offboard
