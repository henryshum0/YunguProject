#include "offboard/waypoint_handler.hpp"

#include <cmath>

namespace offboard
{

WaypointHandler::WaypointHandler(rclcpp::Node &node,
                                 const std::string &buffer_topic,
                                 double reached_dist,
                                 double hold_time,
                                 LocalPosGetter local_pos_getter)
    : node_(node),
      reached_dist_(reached_dist),
      hold_time_(hold_time),
      local_pos_getter_(std::move(local_pos_getter))
{
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

bool WaypointHandler::advanceToNext()
{
    if (buffer_.empty()) {
        return false;
    }

    current_ = buffer_.front();
    buffer_.pop_front();
    wp_reached_ = false;
    const size_t seq = ++seq_;

    RCLCPP_INFO(node_.get_logger(), "Waypoint #%zu promoted: (%.2f, %.2f, %.2f), "
                "%zu still buffered",
                seq,
                current_->pose.position.x, current_->pose.position.y,
                current_->pose.position.z,
                buffer_.size());
    return true;
}

void WaypointHandler::tick(bool active)
{
    if (!active || !current_ || wp_reached_) {
        return;
    }

    const auto local_pos = local_pos_getter_();
    if (!local_pos || !local_pos->xy_valid) {
        return;
    }

    const double enu_x = local_pos->y;
    const double enu_y = local_pos->x;
    const double dx = enu_x - current_->pose.position.x;
    const double dy = enu_y - current_->pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist <= reached_dist_) {
        wp_reached_ = true;
        RCLCPP_INFO(node_.get_logger(),
                    "Waypoint reached (dist=%.2f m)", dist);
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

void WaypointHandler::skipCurrent()
{
    if (current_) {
        RCLCPP_WARN(node_.get_logger(),
                    "Skipping unreachable current waypoint: (%.2f, %.2f, %.2f)",
                    current_->pose.position.x,
                    current_->pose.position.y,
                    current_->pose.position.z);
    }
    current_.reset();
    wp_reached_ = false;
}

void WaypointHandler::completeCurrent()
{
    current_.reset();
    wp_reached_ = false;
}

}  // namespace offboard
