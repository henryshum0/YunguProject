#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleLand()
{
    px4_->publishOffboardControlMode(true, true, true);

    // Direct PX4 landing (no planner): hold the capture point xy and descend
    // to landing_z_ at landing_vel_.
    const float tx = have_hold_ ? hold_x_ : 0.0f;
    const float ty = have_hold_ ? hold_y_ : 0.0f;
    const float target_z = static_cast<float>(landing_z_);
    // NED descend = positive vz (down).
    px4_->publishSetpoint(tx, ty, target_z,
                          0.0f, 0.0f, static_cast<float>(landing_vel_));

    // Landed per PX4's landing detector.
    if (px4_->isLanded()) {
        px4_->disarm();
        active_goal_ = geometry_msgs::msg::PoseStamped();
        waypoints_->clearPending();
        have_hold_ = false;
        RCLCPP_INFO(get_logger(), "Direct landing complete - back to INIT");
        setState(State::INIT);
    }
}

}  // namespace offboard
