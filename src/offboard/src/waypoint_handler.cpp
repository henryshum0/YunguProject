#include "offboard/waypoint_handler.hpp"

#include <cmath>

namespace offboard
{

WaypointHandler::WaypointHandler(rclcpp::Node &node,
                                 const std::string &buffer_topic,
                                 double reached_dist,
                                 double hold_time,
                                 LocalPosGetter local_pos_getter,
                                 GoalPublisher goal_publisher)
    : node_(node),
      reached_dist_(reached_dist),
      hold_time_(hold_time),
      local_pos_getter_(std::move(local_pos_getter)),
      goal_publisher_(std::move(goal_publisher))
{
    // Reliable so buffered waypoints are never dropped in transit.
    buffer_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
        buffer_topic, rclcpp::QoS(10).reliable(),
        std::bind(&WaypointHandler::waypointCallback, this, std::placeholders::_1));
}

void WaypointHandler::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    buffer_.push_back(*msg);
    RCLCPP_INFO(node_.get_logger(), "Waypoint buffered (#%zu): (%.2f, %.2f, %.2f)",
                buffer_.size(),
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void WaypointHandler::publishNextWaypoint()
{
    if (buffer_.empty()) {
        return;
    }

    current_ = buffer_.front();
    buffer_.pop_front();
    wp_reached_ = false;
    const size_t seq = ++seq_;

    // Forward this single waypoint to SUPER (world frame, ENU), which plans to
    // it. /goal_pose is reserved for exactly this.
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_.now();
    goal.header.frame_id = current_->header.frame_id.empty()
                               ? "world" : current_->header.frame_id;
    goal.pose = current_->pose;
    goal_publisher_(goal);

    RCLCPP_INFO(node_.get_logger(), "Waypoint #%zu sent to SUPER: (%.2f, %.2f, %.2f), "
                "%zu still buffered",
                seq,
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                buffer_.size());
}

void WaypointHandler::tick(bool active)
{
    // Run during flight states only (never during takeoff/landing). Checking
    // PLANNER as well lets a reached waypoint advance to the next goal
    // immediately, without waiting for SUPER to drop out of the planner.
    if (!active) {
        return;
    }

    // A waypoint is being pursued (already sent to SUPER).
    if (current_) {
        const auto local_pos = local_pos_getter_();
        // Wait for the drone to reach the waypoint (horizontal distance).
        if (!wp_reached_ && local_pos && local_pos->xy_valid) {
            // PX4 local position is NED; convert to ENU (planner convention).
            const double enu_x = local_pos->y;
            const double enu_y = local_pos->x;
            const double dx = enu_x - current_->pose.position.x;
            const double dy = enu_y - current_->pose.position.y;
            const double dist = std::hypot(dx, dy);
            if (dist <= reached_dist_) {
                wp_reached_ = true;
                wp_reached_t_ = node_.now();
                RCLCPP_INFO(node_.get_logger(),
                            "Waypoint reached (dist=%.2f m) - holding %.1f s",
                            dist, hold_time_);
            }
        }
        // Hold at the reached waypoint, then advance to the next one.
        // NOTE: gate the time subtraction behind wp_reached_ — wp_reached_t_
        // is only valid once the waypoint has been reached, otherwise the
        // rclcpp Time subtraction throws "can't subtract times with different
        // time sources" (default-constructed Time uses the system clock).
        const bool hold_elapsed = wp_reached_ &&
            (node_.now() - wp_reached_t_).seconds() >= hold_time_;
        if (hold_elapsed) {
            current_.reset();
            wp_reached_ = false;
            RCLCPP_INFO(node_.get_logger(), "Waypoint hold complete - advancing");
            if (!buffer_.empty()) {
                publishNextWaypoint();
            }
        }
        return;
    }

    // No active waypoint: start the next one if any are buffered.
    if (!buffer_.empty()) {
        publishNextWaypoint();
    }
}

size_t WaypointHandler::clearPending()
{
    const size_t pending = buffer_.size();
    buffer_.clear();
    current_.reset();
    wp_reached_ = false;
    return pending;
}

}  // namespace offboard
