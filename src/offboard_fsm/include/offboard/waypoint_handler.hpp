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

/** Owns the buffered waypoints and reach detection for the FSM. */
class WaypointHandler
{
public:
    using LocalPosGetter = std::function<std::shared_ptr<const px4_msgs::msg::VehicleLocalPosition>()>;

    explicit WaypointHandler(rclcpp::Node &node,
                             const std::string &buffer_topic,
                             double reached_dist,
                             double hold_time,
                             LocalPosGetter local_pos_getter);

    void tick(bool active);

    bool advanceToNext();

    size_t clearPending();

    void skipCurrent();
    void completeCurrent();

    const std::deque<geometry_msgs::msg::PoseStamped> &buffered() const { return buffer_; }
    const std::optional<geometry_msgs::msg::PoseStamped> &current() const { return current_; }

    bool hasPendingGoal() const { return !buffer_.empty(); }

    const std::optional<geometry_msgs::msg::PoseStamped> &currentGoal() const { return current_; }

    std::optional<geometry_msgs::msg::PoseStamped> nextGoal() const
    {
        if (buffer_.empty()) {
            return std::nullopt;
        }
        return buffer_.front();
    }

    bool hasReachedCurrent() const { return wp_reached_; }

private:
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Node &node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr buffer_sub_;
    double reached_dist_;
    double hold_time_;
    LocalPosGetter local_pos_getter_;

    std::deque<geometry_msgs::msg::PoseStamped> buffer_;
    std::optional<geometry_msgs::msg::PoseStamped> current_;
    bool wp_reached_{false};
    size_t seq_{0};
};

}  // namespace offboard
