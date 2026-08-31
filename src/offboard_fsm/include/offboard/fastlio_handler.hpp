#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace offboard
{

/**
 * @brief Bridges FAST-LIO odometry (ENU) into PX4 EKF2 as external vision.
 *
 * Subscribes to FAST-LIO's /Odometry (nav_msgs/Odometry, frame camera_init →
 * body, ENU) and republishes it as a PX4 /fmu/in/vehicle_visual_odometry
 * (px4_msgs/VehicleOdometry) in NED. This is the C++ replacement for the
 * legacy scripts/fastlio_px4_bridge.py.
 *
 * Conversions (identical to the old Python bridge):
 *   - position:  (y, x, -z)
 *   - velocity:  (vy, vx, -vz)
 *   - attitude:  q_ned = q_enu_to_ned * q_enu,
 *                q_enu_to_ned = (w=0, x=√2/2, y=√2/2, z=0)
 *   - pose_frame = POSE_FRAME_NED, velocity_frame = VELOCITY_FRAME_NED
 *   - covariance diagonals with fallbacks (position 0.01, orientation 0.001,
 *     velocity 0.01) when the incoming covariance is all-zero/non-finite.
 */
class FastLioHandler
{
public:
    explicit FastLioHandler(rclcpp::Node &node,
                            const std::string &odom_topic,
                            const std::string &ev_topic);

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// Extract three variances from a 36-element covariance array at the given
    /// diagonal indices; fall back to `fallback` when zero/non-finite.
    static std::array<float, 3> extractVar(const std::array<double, 36> &cov,
                                           std::array<int, 3> idx, float fallback);

    rclcpp::Node &node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
};

}  // namespace offboard
