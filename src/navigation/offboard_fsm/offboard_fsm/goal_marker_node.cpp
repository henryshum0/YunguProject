#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "offboard_fsm/srv/queue_waypoints.hpp"

namespace offboard
{

/**
 * @brief Accept user waypoints, buffer + mark them, and forward to the offboard
 *        waypoint buffer.
 *
 * Owns the RViz goal intake and its visualisation:
 *   - subscribes /waypoint_pose (geometry_msgs/PoseStamped) — the RViz
 *     "2D Goal Pose" tool re-targeted to /waypoint_pose;
 *   - queues accepted waypoints through the offboard waypoint-buffer service;
 *   - publishes the buffered/current waypoints as markers on /waypoint_markers
 *     so RViz shows the pending route.
 *
 * /goal_pose is left untouched — it is reserved exclusively for the offboard
 * state machine to hand goals to SUPER one at a time.
 */
class GoalMarkerNode : public rclcpp::Node
{
public:
    explicit GoalMarkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("goal_marker", options)
    {
        waypoint_topic_ = declare_parameter("waypoint_topic", waypoint_topic_);
        waypoint_queue_service_ =
            declare_parameter("waypoint_queue_service", waypoint_queue_service_);
        marker_topic_ = declare_parameter("waypoint_marker_topic", marker_topic_);
        marker_rate_ = declare_parameter("waypoint_marker_rate", marker_rate_);
        goal_height_ = declare_parameter("goal_height", goal_height_);
        max_buffered_ = static_cast<std::size_t>(
            declare_parameter("waypoint_buffer_max", static_cast<int>(max_buffered_)));

        waypoint_queue_client_ = create_client<offboard_fsm::srv::QueueWaypoints>(
            waypoint_queue_service_);

        // Waypoint markers (latched + regular republish).
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, rclcpp::QoS(1).transient_local());

        // Waypoints are published by the user (RViz "2D Goal Pose" re-targeted to
        // /waypoint_pose); it publishes reliable/volatile — accept it.
        waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            waypoint_topic_, rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile(),
            std::bind(&GoalMarkerNode::waypointCallback, this, std::placeholders::_1));

        // Marker refresh timer.
        const auto marker_period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / std::max(marker_rate_, 1.0)));
        marker_timer_ = create_wall_timer(
            marker_period, std::bind(&GoalMarkerNode::markerTimerCallback, this));

        RCLCPP_INFO(get_logger(),
                    "Goal marker node started. %s -> %s, markers on %s",
                    waypoint_topic_.c_str(), waypoint_queue_service_.c_str(),
                    marker_topic_.c_str());
    }

private:
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Waypoint received: (%.2f, %.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // A "2D Goal Pose" from RViz carries no meaningful height (z ~ 0). Stamp
        // the configured goal_height so the route is flown at the target altitude.
        auto wp = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
        if (std::abs(wp->pose.position.z) < 1e-3 && goal_height_ > 0.0) {
            wp->pose.position.z = goal_height_;
        }

        if (!waypoint_queue_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Waypoint queue service %s is unavailable",
                        waypoint_queue_service_.c_str());
            return;
        }
        auto request = std::make_shared<offboard_fsm::srv::QueueWaypoints::Request>();
        request->waypoints.push_back(*wp);
        waypoint_queue_client_->async_send_request(
            request,
            [this, wp](rclcpp::Client<offboard_fsm::srv::QueueWaypoints>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (!response->success || response->queued_count != 1U) {
                        RCLCPP_WARN(get_logger(), "Waypoint queue request failed: %s",
                                    response->message.c_str());
                        return;
                    }
                    buffer_.push_back(*wp);
                    if (buffer_.size() > max_buffered_) {
                        buffer_.pop_front();
                    }
                } catch (const std::exception &error) {
                    RCLCPP_WARN(get_logger(), "Waypoint queue request threw: %s", error.what());
                }
            });
    }

    void markerTimerCallback()
    {
        visualization_msgs::msg::MarkerArray markers;

        // Clear stale markers from the previous publish.
        visualization_msgs::msg::Marker clear;
        clear.header.stamp = now();
        clear.header.frame_id = "world";
        clear.ns = "";
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);

        // Green spheres for the buffered (pending) waypoints.
        int i = 0;
        for (const auto &wp : buffer_) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = now();
            m.header.frame_id = "world";
            m.ns = "waypoint";
            m.id = i++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = wp.pose.position;
            m.scale.x = 2.0f;
            m.scale.y = 2.0f;
            m.scale.z = 2.0f;
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            markers.markers.push_back(m);
        }

        // Yellow sphere for the most recent (newest) goal.
        if (!buffer_.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = now();
            m.header.frame_id = "world";
            m.ns = "active";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = buffer_.back().pose.position;
            m.scale.x = 2.0f;
            m.scale.y = 2.0f;
            m.scale.z = 2.0f;
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            markers.markers.push_back(m);
        }

        // Cyan line connecting the route, in order (only drawn with >= 2 points).
        {
            visualization_msgs::msg::Marker line;
            line.header.stamp = now();
            line.header.frame_id = "world";
            line.ns = "route";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 1.0f;  // line width [m]
            line.color.r = 0.0f;
            line.color.g = 1.0f;
            line.color.b = 1.0f;
            line.color.a = 1.0f;
            for (const auto &wp : buffer_) {
                line.points.push_back(wp.pose.position);
            }
            if (line.points.size() >= 2) {
                markers.markers.push_back(line);
            }
        }

        marker_pub_->publish(markers);
    }

    std::string waypoint_topic_{"/waypoint_pose"};
    std::string waypoint_queue_service_{"/waypoint_buffer"};
    std::string marker_topic_{"/waypoint_markers"};
    double marker_rate_{10.0};
    double goal_height_{5.0};
    std::size_t max_buffered_{200};

    rclcpp::Client<offboard_fsm::srv::QueueWaypoints>::SharedPtr waypoint_queue_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    std::deque<geometry_msgs::msg::PoseStamped> buffer_;
};

}  // namespace offboard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::GoalMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
