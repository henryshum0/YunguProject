#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleTakeoff()
{
    px4_->publishOffboardControlMode(true, true, true);

    // Capture the takeoff origin (NED) on entry. Direct PX4 position control:
    // hold xy, climb to default_height.
    if (!have_takeoff_goal_) {
        const auto local_pos = px4_->getLocalPosition();
        if (!local_pos) {
            publishHold();
            return;
        }
        takeoff_target_x_ = local_pos->x;                          // NED
        takeoff_target_y_ = local_pos->y;
        takeoff_target_z_ = static_cast<float>(-default_height_);  // NED up = negative
        have_takeoff_goal_ = true;
        RCLCPP_INFO(get_logger(),
                    "Direct takeoff: climb to NED z=%.2f (height %.1f m)",
                    takeoff_target_z_, default_height_);
    }

    // Wait briefly for OFFBOARD mode / stream to engage before climbing.
    if (stateElapsedSec() < 0.3) {
        publishHold();
        return;
    }

    // Direct PX4 vertical climb: command the target altitude with an upward
    // velocity feed-forward (NED up = negative vz).
    const float vz = static_cast<float>(-takeoff_vel_);
    px4_->publishSetpoint(takeoff_target_x_, takeoff_target_y_, takeoff_target_z_,
                          0.0f, 0.0f, vz);

    // Takeoff complete once near the target altitude.
    const auto local_pos = px4_->getLocalPosition();
    if (local_pos && std::abs(local_pos->z - takeoff_target_z_) < 0.5) {
        RCLCPP_INFO(get_logger(), "Takeoff complete at z=%.2f m - entering IDLE",
                    takeoff_target_z_);
        // Set the IDLE hold to the takeoff altitude so the drone hovers there
        // instead of falling back to the default (0,0,0) hold.
        captureHold();
        have_takeoff_goal_ = false;
        setState(State::IDLE);
    }
}

}  // namespace offboard
