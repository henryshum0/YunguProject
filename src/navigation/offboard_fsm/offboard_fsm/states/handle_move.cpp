#include "offboard/offboard.hpp"

namespace offboard
{

void OffboardNode::handleMove()
{
    updatePlannerActivity();

    if (land_requested_) {
        land_requested_ = false;
        captureHold();
        setState(State::LAND);
        return;
    }

    if (const auto status = super_->getGoalStatus(); status &&
        (status->status == super_planner::msg::GoalStatus::STATUS_REACHED ||
         status->status == super_planner::msg::GoalStatus::STATUS_CLOSE ||
         status->status == super_planner::msg::GoalStatus::STATUS_STUCK)) {
        captureHold();
        setState(State::IDLE);
        return;
    }

    if (super_->isPlannerFail()) {
        captureHold();
        setState(State::IDLE);
        return;
    }

    if (!planner_active_) {
        captureHold();
        publishHold();
        return;
    }

    const auto cmd = super_->getLatestCommand();
    if (!cmd) {
        captureHold();
        publishHold();
        return;
    }

    const auto &c = *cmd;
    float nx, ny, nz, vx, vy, vz, ax, ay, az;
    frame::enuToNed(c.position.x, c.position.y, c.position.z, nx, ny, nz);
    frame::enuToNed(c.velocity.x, c.velocity.y, c.velocity.z, vx, vy, vz);
    frame::enuToNed(c.acceleration.x, c.acceleration.y, c.acceleration.z, ax, ay, az);
    const float yaw_ned = static_cast<float>(frame::enuYawToNed(c.yaw));
    const float yawspeed_ned = static_cast<float>(frame::enuYawRateToNed(c.yaw_dot));
    px4_->publishSetpoint(nx, ny, nz, vx, vy, vz, yaw_ned, yawspeed_ned, ax, ay, az);

    if (waypoints_->hasReachedCurrent()) {
        captureHold();
        setState(State::IDLE);
    }
}

}  // namespace offboard
