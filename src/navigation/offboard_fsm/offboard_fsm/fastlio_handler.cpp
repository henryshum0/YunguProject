#include "offboard/fastlio_handler.hpp"

#include <cmath>

#include "offboard/frame_conversion.hpp"

namespace offboard
{

namespace
{
// ENU -> NED reference-frame rotation quaternion (w, x, y, z): a +90° rotation
// about the +X axis so that (East, North, Up) maps to (North, East, Down).
constexpr double kQEnuToNed[4] = {0.0, 0.70710678118654752440,  // √2 / 2
                                  0.70710678118654752440, 0.0};

/// Normalize a quaternion (w,x,y,z); returns the input unchanged when the norm
/// is (numerically) zero.
void normalizeQuat(double &w, double &x, double &y, double &z)
{
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-12) {
        return;
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;
}
}  // namespace

FastLioHandler::FastLioHandler(rclcpp::Node &node,
                               const std::string &odom_topic,
                               const std::string &ev_topic)
    : node_(node)
{
    auto qos_be = rclcpp::QoS(5).best_effort();
    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_be,
        std::bind(&FastLioHandler::odomCallback, this, std::placeholders::_1));
    ev_pub_ = node_.create_publisher<px4_msgs::msg::VehicleOdometry>(ev_topic, qos_be);

    RCLCPP_INFO(node_.get_logger(), "fastlio_handler: %s -> %s",
                odom_topic.c_str(), ev_topic.c_str());
}

void FastLioHandler::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    px4_msgs::msg::VehicleOdometry ev;

    const auto &st = msg->header.stamp;
    const std::uint64_t us = static_cast<std::uint64_t>(st.sec) * 1000000ULL +
                             static_cast<std::uint64_t>(st.nanosec) / 1000ULL;
    ev.timestamp = us;
    ev.timestamp_sample = us;

    ev.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    // ENU -> NED position: (x_e, y_e, z_e) -> (y_e, x_e, -z_e).
    ev.position[0] = static_cast<float>(msg->pose.pose.position.y);
    ev.position[1] = static_cast<float>(msg->pose.pose.position.x);
    ev.position[2] = static_cast<float>(-msg->pose.pose.position.z);

    // ENU -> NED attitude: q_ned = q_enu_to_ned * q_enu.
    const auto &q = msg->pose.pose.orientation;
    double qw, qx, qy, qz;
    frame::quatMul(kQEnuToNed[0], kQEnuToNed[1], kQEnuToNed[2], kQEnuToNed[3],
                   q.w, q.x, q.y, q.z, qw, qx, qy, qz);
    normalizeQuat(qw, qx, qy, qz);
    ev.q[0] = static_cast<float>(qw);
    ev.q[1] = static_cast<float>(qx);
    ev.q[2] = static_cast<float>(qy);
    ev.q[3] = static_cast<float>(qz);

    ev.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    // ENU -> NED velocity: same swap + z-negate.
    ev.velocity[0] = static_cast<float>(msg->twist.twist.linear.y);
    ev.velocity[1] = static_cast<float>(msg->twist.twist.linear.x);
    ev.velocity[2] = static_cast<float>(-msg->twist.twist.linear.z);
    // Angular velocity is reported in the body frame (BODY_FRD); FAST-LIO's
    // twist.angular is already body-frame, so copy it through unchanged (the
    // PX4 field is declared as VELOCITY_FRAME_BODY_FRD).
    ev.angular_velocity[0] = static_cast<float>(msg->twist.twist.angular.x);
    ev.angular_velocity[1] = static_cast<float>(msg->twist.twist.angular.y);
    ev.angular_velocity[2] = static_cast<float>(msg->twist.twist.angular.z);

    ev.position_variance = extractVar(msg->pose.covariance, {0, 7, 14}, 0.01f);
    ev.orientation_variance = extractVar(msg->pose.covariance, {21, 28, 35}, 0.001f);
    ev.velocity_variance = extractVar(msg->twist.covariance, {0, 7, 14}, 0.01f);

    ev.reset_counter = 0;
    ev.quality = 0;
    ev_pub_->publish(ev);
}

std::array<float, 3> FastLioHandler::extractVar(const std::array<double, 36> &cov,
                                                std::array<int, 3> idx,
                                                float fallback)
{
    std::array<float, 3> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        const double v = cov[idx[i]];
        if (!std::isfinite(v) || std::abs(v) < 1e-15) {
            out[i] = fallback;
        } else {
            out[i] = static_cast<float>(std::max(v, 0.0));
        }
    }
    return out;
}

}  // namespace offboard
