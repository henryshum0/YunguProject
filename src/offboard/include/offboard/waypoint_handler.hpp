#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

namespace offboard
{

/**
 * @brief Owns the buffered-waypoint following for the offboard state machine.
 *
 * Subscribes to the waypoint buffer topic (/waypoint_buffer, fed by the goal
 * marker node) and manages the buffered waypoints one at a time: exposes the
 * currently-pursued waypoint, waits until the drone is within `reached_dist`
 * horizontally, holds `hold_time`, then advances to the next buffered one.
 *
 * It is decoupled from OffboardNode and the other handlers through an injected
 * local-position getter (for the reach check; the NED -> ENU conversion
 * happens internally). It does NOT publish to SUPER — the state machine is the
 * sole /goal_pose publisher and forwards the current waypoint to SUPER when in
 * MOVE.
 */
class WaypointHandler
{
public:
    /// Provides the latest PX4 local position (NED) for reach checks.
    using LocalPosGetter = std::function<std::shared_ptr<const px4_msgs::msg::VehicleLocalPosition>()>;

    explicit WaypointHandler(rclcpp::Node &node,
                             const std::string &buffer_topic,
                             double reached_dist,
                             double hold_time,
                             LocalPosGetter local_pos_getter);

    /// Advance waypoint following. Called on every state-machine tick; gated
    /// internally to the flight states (IDLE / MOVE) via `active`.
    void tick(bool active);

    /// Discard all buffered + in-flight waypoints (on planner exit). Returns
    /// the number of buffered waypoints that were discarded.
    size_t clearPending();

    // --- Read access for visualization / logging ---
    const std::deque<geometry_msgs::msg::PoseStamped> &buffered() const { return buffer_; }
    const std::optional<geometry_msgs::msg::PoseStamped> &current() const { return current_; }

    /// True when there is a goal to fly: an in-flight waypoint is being
    /// pursued or at least one waypoint is still buffered. Used by the state
    /// machine to decide whether IDLE can transition to MOVE.
    bool hasPendingGoal() const { return current_.has_value() || !buffer_.empty(); }

    /// The currently-pursued waypoint goal (if any). Empty when idle.
    const std::optional<geometry_msgs::msg::PoseStamped> &currentGoal() const { return current_; }

private:
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishNextWaypoint();

    rclcpp::Node &node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr buffer_sub_;
    double reached_dist_;
    double hold_time_;
    LocalPosGetter local_pos_getter_;

    std::deque<geometry_msgs::msg::PoseStamped> buffer_;
    std::optional<geometry_msgs::msg::PoseStamped> current_;
    bool wp_reached_{false};
    rclcpp::Time wp_reached_t_;
    size_t seq_{0};
};

}  // namespace offboard
