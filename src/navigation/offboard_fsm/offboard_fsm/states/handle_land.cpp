#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleLand()
{
    // Hand flight control back to PX4.  Do not publish OffboardControlMode or
    // trajectory setpoints in LAND: either one can immediately pull PX4 out
    // of AUTO_LAND or override its landing controller.
    if (!px4_->isLanded()) {
        const double elapsed = land_command_requested_
                                   ? (now() - last_land_t_).seconds()
                                   : land_retry_delay_;
        if (elapsed >= land_retry_delay_) {
            px4_->land();
            last_land_t_ = now();
            land_command_requested_ = true;
            RCLCPP_INFO(get_logger(), "Requested PX4 native landing mode");
        }
        return;
    }

    // PX4 requires ground contact before a normal disarm.  Keep retrying a
    // command after its dedicated landing detector reports touchdown; a
    // VehicleCommand uses best-effort QoS, so a single command is not enough.
    if (!px4_->isDisarmed()) {
        const double elapsed = disarm_requested_
                                   ? (now() - last_disarm_t_).seconds()
                                   : disarm_retry_delay_;
        if (elapsed >= disarm_retry_delay_) {
            px4_->disarm();
            last_disarm_t_ = now();
            disarm_requested_ = true;
            RCLCPP_INFO(get_logger(), "Landed; requesting PX4 disarm");
        }
        return;
    }

    active_goal_ = geometry_msgs::msg::PoseStamped();
    waypoints_->clearPending();
    have_hold_ = false;
    have_takeoff_goal_ = false;
    land_requested_ = false;
    RCLCPP_INFO(get_logger(), "Landing and disarm confirmed - back to INIT");
    setState(State::INIT);
}

}  // namespace offboard
