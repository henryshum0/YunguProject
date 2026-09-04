#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleInit()
{
    // Stream origin so PX4 sees the offboard stream (and so OFFBOARD mode is
    // accepted) before we arm.
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);

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
        px4_->setOffboardMode();
        return;
    }

    if (takeoff_requested_) {
        RCLCPP_INFO(get_logger(), "Takeoff requested - entering ARMING");
        takeoff_requested_ = false;
        arm_retry_count_ = 0;
        setState(State::ARMING);
    }
}

}  // namespace offboard
