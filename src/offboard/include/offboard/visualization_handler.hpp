#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace offboard
{

/**
 * @brief Owns the waypoint-route visualization for the offboard state machine.
 *
 * Owns the waypoint marker publisher (/waypoint_markers) and its periodic
 * republish timer, and builds the MarkerArray (buffered waypoints as green
 * spheres, the currently-pursued waypoint as a yellow sphere, and the
 * connecting route as a cyan line). The latest waypoint state is pulled from
 * the state machine each tick through the route provider callback.
 */
class VisualizationHandler
{
public:
    /// Fetch the current waypoint state (buffered + active waypoint) from the
    /// state machine. Called on every marker timer tick.
    using RouteProvider = std::function<void(
        std::deque<geometry_msgs::msg::PoseStamped> &buffered,
        std::optional<geometry_msgs::msg::PoseStamped> &current)>;

    explicit VisualizationHandler(rclcpp::Node &node,
                                  const std::string &marker_topic,
                                  double marker_rate_hz,
                                  RouteProvider route_provider);

    // --- Outgoing (state machine -> RViz) ---
    /// Rebuild and publish the waypoint markers from the current route state.
    void publishWaypointMarkers();

private:
    void markerTimerCallback();

    rclcpp::Node &node_;
    RouteProvider route_provider_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace offboard
