#include "offboard/visualization_handler.hpp"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace offboard
{

VisualizationHandler::VisualizationHandler(rclcpp::Node &node,
                                           const std::string &marker_topic,
                                           double marker_rate_hz,
                                           RouteProvider route_provider)
    : node_(node),
      route_provider_(std::move(route_provider))
{
    // MarkerArray, latched + regular republish.
    waypoint_marker_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic, rclcpp::QoS(1).transient_local());

    const auto marker_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(marker_rate_hz, 1.0)));
    marker_timer_ = node_.create_wall_timer(
        marker_period, std::bind(&VisualizationHandler::markerTimerCallback, this));
}

void VisualizationHandler::markerTimerCallback()
{
    publishWaypointMarkers();
}

void VisualizationHandler::publishWaypointMarkers()
{
    // Pull the latest route state from the state machine.
    std::deque<geometry_msgs::msg::PoseStamped> buffered;
    std::optional<geometry_msgs::msg::PoseStamped> current;
    route_provider_(buffered, current);

    visualization_msgs::msg::MarkerArray markers;

    // Clear stale markers from the previous publish.
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = node_.now();
    clear.header.frame_id = "world";
    clear.ns = "";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    // Green spheres for the buffered (pending) waypoints.
    int i = 0;
    for (const auto &wp : buffered) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = node_.now();
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

    // Yellow sphere for the waypoint currently being pursued.
    if (current) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = node_.now();
        m.header.frame_id = "world";
        m.ns = "active";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = current->pose.position;
        m.scale.x = 2.0f;
        m.scale.y = 2.0f;
        m.scale.z = 2.0f;
        m.color.r = 1.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        markers.markers.push_back(m);
    }

    // Cyan line connecting the route: the active waypoint (if any) then the
    // buffered waypoints, in order. Only drawn with >= 2 points.
    {
        visualization_msgs::msg::Marker line;
        line.header.stamp = node_.now();
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
        if (current) {
            line.points.push_back(current->pose.position);
        }
        for (const auto &wp : buffered) {
            line.points.push_back(wp.pose.position);
        }
        if (line.points.size() >= 2) {
            markers.markers.push_back(line);
        }
    }

    waypoint_marker_pub_->publish(markers);
}

}  // namespace offboard
