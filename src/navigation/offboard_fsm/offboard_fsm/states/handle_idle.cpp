#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleIdle()
{
    updatePlannerActivity();
    if (!have_hold_) {
        captureHold();
    }
    publishIdleHold();

    if (land_requested_) {
        land_requested_ = false;
        captureHold();
        setState(State::LAND);
        return;
    }

    if (planner_reset_in_flight_) {
        return;
    }

    if (!super_->isPlannerWaitGoal()) {
        const bool cooldown_elapsed =
            (now() - last_planner_reset_t_).seconds() >= planner_reset_delay_;
        if (cooldown_elapsed) {
            restartPlanner();
        }
        return;
    }

    bool terminal_status = false;
    if (const auto status = super_->getGoalStatus()) {
        switch (status->status) {
            case super_planner::msg::GoalStatus::STATUS_REACHED:
                waypoints_->completeCurrent();
                stuck_recovery_attempted_ = false;
                terminal_status = true;
                break;

            case super_planner::msg::GoalStatus::STATUS_CLOSE:
                RCLCPP_INFO(get_logger(), "Goal is close; skipping to the next goal");
                waypoints_->completeCurrent();
                stuck_recovery_attempted_ = false;
                terminal_status = true;
                break;

            case super_planner::msg::GoalStatus::STATUS_STUCK:
                waypoints_->skipCurrent();
                if (stuck_recovery_attempted_ && waypoints_->hasPendingGoal()) {
                    RCLCPP_WARN(get_logger(),
                                "Planner is stuck on consecutive goals; clearing the goal buffer");
                    waypoints_->clearPending();
                    stuck_recovery_attempted_ = false;
                    super_->clearGoalStatus();
                    return;
                }
                stuck_recovery_attempted_ = waypoints_->hasPendingGoal();
                terminal_status = true;
                break;

            default:
                break;
        }
        super_->clearGoalStatus();
    }

    if (terminal_status) {
        restartPlanner();
        return;
    }

    if (!waypoints_->hasPendingGoal()) {
        if (!waypoints_->currentGoal().has_value()) {
            stuck_recovery_attempted_ = false;
        }
        return;
    }

    if (!headingOk()) {
        return;
    }

    if (publishCurrentGoal()) {
        RCLCPP_INFO(get_logger(), "New goal - entering MOVE");
        setState(State::MOVE);
    }
}

}  // namespace offboard
