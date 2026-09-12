#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleInit()
{
    // Stream a local hold so PX4 sees the offboard stream before arming.  On a
    // later takeoff this must be the landed position, not the original world
    // origin, otherwise PX4 can be commanded sideways along the ground.
    if (!have_hold_ && px4_->hasValidPosition()) {
        captureHold();
    }
    publishHold();

    if (super_->isLioError() || super_->isPlannerFail()) {
        RCLCPP_WARN(get_logger(), "Planner or odometry is not ready");
        return;
    }

    // If the vehicle is already airborne in OFFBOARD mode (e.g. the FSM was
    // restarted mid-flight) and odometry + planner are ready, resume directly
    // in IDLE instead of waiting for a takeoff command / going through
    // ARMING->TAKEOFF.
    if (px4_->isOffboard() && super_->isPlannerReady() && px4_->isInAir()) {
        RCLCPP_INFO(get_logger(),
                    "Vehicle already OFFBOARD and airborne - resuming in IDLE");
        captureHold();
        setState(State::IDLE);
        return;
    }

    // Wait until the whole system is ready (odom + fastlio + planner +
    // landed & disarmed), then arm OFFBOARD mode and await takeoff.
    if (!systemReady()) {
        return;
    }

    if (!px4_->isOffboard()) {
        offboard_ready_ = false;
        px4_->setOffboardMode();
        return;
    }

    if (!offboard_ready_) {
        offboard_ready_ = true;
        offboard_ready_t_ = now();
        RCLCPP_INFO(get_logger(),
                    "OFFBOARD confirmed; waiting %.1f s before arming", arm_wait_);
    }

    if (takeoff_requested_) {
        if ((now() - offboard_ready_t_).seconds() < arm_wait_) {
            return;
        }
        RCLCPP_INFO(get_logger(), "Takeoff requested - entering ARMING");
        arm_retry_count_ = 0;
        setState(State::ARMING);
    }
}

}  // namespace offboard
