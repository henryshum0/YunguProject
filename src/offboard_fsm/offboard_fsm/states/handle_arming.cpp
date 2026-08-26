#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleArming()
{
    px4_->publishSetpoint(0.0f, 0.0f, 0.0f);

    // Heartbeat loss (armed but no longer healthy) -> back to INIT.
    if (super_->isLioError() || !super_->isPlannerReady()) {
        RCLCPP_WARN(get_logger(), "Heartbeat/link failed during ARMING - back to INIT");
        setState(State::INIT);
        return;
    }

    if (px4_->isArmed()) {
        RCLCPP_INFO(get_logger(), "Vehicle confirmed ARMED");
        // The takeoff origin is captured in handleTakeoff (direct PX4 climb).
        have_takeoff_goal_ = false;
        setState(State::TAKEOFF);
        return;
    }

    // Retry arming with a delay between attempts. Guard against a
    // default-constructed stamp (system-clock source) which cannot be
    // subtracted from the ROS-time `now()`.
    const double elapsed = last_arm_t_.nanoseconds() != 0
                               ? (now() - last_arm_t_).seconds()
                               : arm_retry_delay_;
    if (elapsed >= arm_retry_delay_) {
        px4_->arm();
        last_arm_t_ = now();
        ++arm_retry_count_;
        RCLCPP_INFO(get_logger(), "Arm attempt #%d/%d",
                    arm_retry_count_, arm_retry_max_);
        if (arm_retry_count_ >= arm_retry_max_) {
            RCLCPP_WARN(get_logger(), "Arm retry limit reached - back to INIT");
            setState(State::INIT);
            return;
        }
    }
}

}  // namespace offboard
