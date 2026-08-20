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
 * marker node) and flies the buffered waypoints one at a time: publishes the
 * current waypoint to SUPER through the injected goal publisher, waits until
 * the drone is within `reached_dist` horizontally, holds `hold_time`, then
 * advances to the next buffered one.
 *
 * It is decoupled from OffboardNode and the other handlers through two
 * injected callbacks: a local-position getter (for the reach check; the NED ->
 * ENU conversion happens internally) and a goal publisher (forwards each
 * waypoint to SUPER).
 */
class WaypointHandler
{
public:
    /// Provides the latest PX4 local position (NED) for reach checks.
    using LocalPosGetter = std::function<std::shared_ptr<const px4_msgs::msg::VehicleLocalPosition>()>;
    /// Forwards a waypoint goal to SUPER.
    using GoalPublisher = std::function<void(const geometry_msgs::msg::PoseStamped &)>;

    explicit WaypointHandler(rclcpp::Node &node,
                             const std::string &buffer_topic,
                             double reached_dist,
                             double hold_time,
                             LocalPosGetter local_pos_getter,
                             GoalPublisher goal_publisher);

    /// Advance waypoint following. Called on every state-machine tick; gated
    /// internally to the flight states (IDLE / PLANNER) via `active`.
    void tick(bool active);

    /// Discard all buffered + in-flight waypoints (on planner exit). Returns
    /// the number of buffered waypoints that were discarded.
    size_t clearPending();

    // --- Read access for visualization / logging ---
    const std::deque<geometry_msgs::msg::PoseStamped> &buffered() const { return buffer_; }
    const std::optional<geometry_msgs::msg::PoseStamped> &current() const { return current_; }

private:
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishNextWaypoint();

    rclcpp::Node &node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr buffer_sub_;
    double reached_dist_;
    double hold_time_;
    LocalPosGetter local_pos_getter_;
    GoalPublisher goal_publisher_;

    std::deque<geometry_msgs::msg::PoseStamped> buffer_;
    std::optional<geometry_msgs::msg::PoseStamped> current_;
    bool wp_reached_{false};
    rclcpp::Time wp_reached_t_;
    size_t seq_{0};
};

}  // namespace offboard
