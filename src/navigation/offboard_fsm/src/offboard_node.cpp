#include <cmath>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "offboard_fsm/frame_conversion.hpp"
#include "offboard_fsm/srv/clear_waypoints.hpp"
#include "offboard_fsm/srv/queue_waypoints.hpp"

namespace
{
using QueueWaypoints = offboard_fsm::srv::QueueWaypoints;
using ClearWaypoints = offboard_fsm::srv::ClearWaypoints;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class OffboardNode final : public rclcpp::Node
{
public:
  OffboardNode() : Node("offboard")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    warmup_sec_ = declare_parameter<double>("warmup_sec", 1.0);
    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 5.0);
    takeoff_speed_mps_ = declare_parameter<double>("takeoff_speed_mps", 0.6);
    takeoff_tolerance_m_ = declare_parameter<double>("takeoff_tolerance_m", 0.3);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    goal_accept_timeout_sec_ = declare_parameter<double>("goal_accept_timeout_sec", 8.0);
    arm_retry_sec_ = declare_parameter<double>("arm_retry_sec", 1.0);

    local_position_topic_ = declare_parameter<std::string>(
      "local_position_topic", "/fmu/out/vehicle_local_position");
    vehicle_status_topic_ = declare_parameter<std::string>(
      "vehicle_status_topic", "/fmu/out/vehicle_status");
    land_detected_topic_ = declare_parameter<std::string>(
      "land_detected_topic", "/fmu/out/vehicle_land_detected");
    ego_goal_topic_ = declare_parameter<std::string>("ego_goal_topic", "/move_base_simple/goal");
    ego_state_topic_ = declare_parameter<std::string>("ego_state_topic", "/ego_planner/state");
    ego_command_topic_ = declare_parameter<std::string>("ego_command_topic", "/position_cmd");
    queue_service_ = declare_parameter<std::string>("queue_service", "/waypoint_buffer");
    clear_service_ = declare_parameter<std::string>("clear_service", "/waypoint_buffer/clear");
    takeoff_service_ = declare_parameter<std::string>("takeoff_service", "/offboard/takeoff");
    land_service_ = declare_parameter<std::string>("land_service", "/offboard/land");
    queue_status_topic_ = declare_parameter<std::string>(
      "queue_status_topic", "/waypoint_buffer/status");
    offboard_mode_topic_ = declare_parameter<std::string>(
      "offboard_mode_topic", "/fmu/in/offboard_control_mode");
    trajectory_setpoint_topic_ = declare_parameter<std::string>(
      "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    vehicle_command_topic_ = declare_parameter<std::string>(
      "vehicle_command_topic", "/fmu/in/vehicle_command");

    if (update_rate_hz_ <= 1.0) {
      throw std::runtime_error("update_rate_hz must be greater than 1 Hz");
    }

    const auto px4_qos = rclcpp::QoS(10).best_effort();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_position_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
        local_position_ = std::move(message);
      });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr message) {
        vehicle_status_ = std::move(message);
      });
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      land_detected_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
        land_detected_ = std::move(message);
      });
    ego_state_sub_ = create_subscription<std_msgs::msg::String>(
      ego_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) { ego_state_ = message->data; });
    ego_command_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      ego_command_topic_, rclcpp::QoS(20).best_effort(),
      [this](quadrotor_msgs::msg::PositionCommand::SharedPtr message) {
        latest_ego_command_ = std::move(message);
        latest_ego_command_time_ = now();
      });

    ego_goal_pub_ = create_publisher<PoseStamped>(ego_goal_topic_, rclcpp::QoS(1).reliable());
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_mode_topic_, px4_qos);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic_, px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      vehicle_command_topic_, px4_qos);
    queue_status_pub_ = create_publisher<nav_msgs::msg::Path>(
      queue_status_topic_, rclcpp::QoS(1).reliable().transient_local());

    queue_service_server_ = create_service<QueueWaypoints>(
      queue_service_, std::bind(&OffboardNode::queue_waypoints, this,
      std::placeholders::_1, std::placeholders::_2));
    clear_service_server_ = create_service<ClearWaypoints>(
      clear_service_, std::bind(&OffboardNode::clear_waypoints, this,
      std::placeholders::_1, std::placeholders::_2));
    takeoff_service_server_ = create_service<std_srvs::srv::Trigger>(
      takeoff_service_, std::bind(&OffboardNode::takeoff, this,
      std::placeholders::_1, std::placeholders::_2));
    land_service_server_ = create_service<std_srvs::srv::Trigger>(
      land_service_, std::bind(&OffboardNode::land, this,
      std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OffboardNode::tick, this));
    state_entered_ = now();
    last_vehicle_command_ = now();
    publish_queue_status();
    RCLCPP_INFO(get_logger(), "EGO offboard adapter ready: goals=%s, commands=%s",
      ego_goal_topic_.c_str(), ego_command_topic_.c_str());
  }

private:
  enum class State { INIT, TAKEOFF, IDLE, MOVE, LAND };

  void tick()
  {
    switch (state_) {
      case State::INIT: handle_init(); break;
      case State::TAKEOFF: handle_takeoff(); break;
      case State::IDLE: handle_idle(); break;
      case State::MOVE: handle_move(); break;
      case State::LAND: handle_land(); break;
    }
  }

  void handle_init()
  {
    publish_offboard_mode();
    ensure_hold();
    publish_hold();
    if (!takeoff_requested_ || !has_valid_position()) {
      return;
    }
    if (state_elapsed() < warmup_sec_) {
      return;
    }
    set_state(State::TAKEOFF);
  }

  void handle_takeoff()
  {
    publish_offboard_mode();
    ensure_hold();
    if (!takeoff_target_set_) {
      takeoff_x_ = hold_x_;
      takeoff_y_ = hold_y_;
      takeoff_z_ = hold_z_ - static_cast<float>(takeoff_height_m_);
      takeoff_target_set_ = true;
      RCLCPP_INFO(get_logger(), "Takeoff target: NED (%.2f, %.2f, %.2f)",
        takeoff_x_, takeoff_y_, takeoff_z_);
    }

    if (time_since(last_vehicle_command_) >= arm_retry_sec_) {
      send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      if (!is_armed()) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      }
      last_vehicle_command_ = now();
    }
    if (!is_armed()) {
      publish_hold();
      return;
    }

    publish_setpoint(takeoff_x_, takeoff_y_, takeoff_z_, 0.0f, 0.0f,
      static_cast<float>(-takeoff_speed_mps_), hold_yaw_);
    if (local_position_ && std::abs(local_position_->z - takeoff_z_) <= takeoff_tolerance_m_) {
      capture_hold();
      takeoff_requested_ = false;
      takeoff_target_set_ = false;
      set_state(State::IDLE);
    }
  }

  void handle_idle()
  {
    publish_offboard_mode();
    ensure_hold();
    publish_hold();
    if (!is_armed()) {
      set_state(State::INIT);
      return;
    }
    if (active_goal_ || queue_.empty()) {
      return;
    }
    active_goal_ = queue_.front();
    queue_.pop_front();
    latest_ego_command_.reset();
    awaiting_execution_ = true;
    execution_observed_ = false;
    ego_goal_pub_->publish(*active_goal_);
    publish_queue_status();
    set_state(State::MOVE);
    RCLCPP_INFO(get_logger(), "Dispatched EGO goal: (%.2f, %.2f, %.2f)",
      active_goal_->pose.position.x, active_goal_->pose.position.y,
      active_goal_->pose.position.z);
  }

  void handle_move()
  {
    publish_offboard_mode();
    if (ego_state_ == "ERROR" || ego_state_ == "EMERGENCY_STOP") {
      abort_route("EGO planner reported " + ego_state_);
      return;
    }
    if (awaiting_execution_ && ego_state_ == "EXEC_TRAJ") {
      awaiting_execution_ = false;
      execution_observed_ = true;
    }
    if (awaiting_execution_ && state_elapsed() > goal_accept_timeout_sec_) {
      abort_route("EGO planner did not accept the goal before timeout");
      return;
    }
    if (execution_observed_ && ego_state_ == "WAIT_TARGET") {
      complete_active_goal();
      return;
    }
    if (!latest_ego_command_ || time_since(latest_ego_command_time_) > command_timeout_sec_) {
      publish_hold();
      if (execution_observed_) {
        abort_route("EGO position command stream timed out");
      }
      return;
    }
    const auto &command = *latest_ego_command_;
    float px, py, pz, vx, vy, vz, ax, ay, az;
    offboard_fsm::frame::enu_to_ned(command.position.x, command.position.y, command.position.z, px, py, pz);
    offboard_fsm::frame::enu_to_ned(command.velocity.x, command.velocity.y, command.velocity.z, vx, vy, vz);
    offboard_fsm::frame::enu_to_ned(command.acceleration.x, command.acceleration.y, command.acceleration.z, ax, ay, az);
    publish_setpoint(px, py, pz, vx, vy, vz,
      static_cast<float>(offboard_fsm::frame::enu_yaw_to_ned(command.yaw)),
      static_cast<float>(offboard_fsm::frame::enu_yaw_rate_to_ned(command.yaw_dot)), ax, ay, az);
  }

  void handle_land()
  {
    if (!land_command_sent_ || time_since(last_vehicle_command_) >= arm_retry_sec_) {
      send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      land_command_sent_ = true;
      last_vehicle_command_ = now();
    }
    if (is_landed()) {
      if (is_armed() && time_since(last_vehicle_command_) >= arm_retry_sec_) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        last_vehicle_command_ = now();
      }
      if (!is_armed()) {
        have_hold_ = false;
        land_command_sent_ = false;
        set_state(State::INIT);
      }
    }
  }

  void queue_waypoints(const std::shared_ptr<QueueWaypoints::Request> request,
                       std::shared_ptr<QueueWaypoints::Response> response)
  {
    if (request->waypoints.empty()) {
      response->success = false;
      response->message = "waypoint queue request must contain at least one waypoint";
      response->queued_count = 0;
      return;
    }
    for (const auto &waypoint : request->waypoints) {
      if (!valid_waypoint(waypoint)) {
        response->success = false;
        response->message = "waypoints must contain finite ENU positions";
        response->queued_count = 0;
        return;
      }
    }
    for (auto waypoint : request->waypoints) {
      waypoint.header.frame_id = frame_id_;
      queue_.push_back(std::move(waypoint));
    }
    response->success = true;
    response->queued_count = static_cast<uint32_t>(request->waypoints.size());
    response->message = "queued " + std::to_string(response->queued_count) + " waypoint(s)";
    publish_queue_status();
  }

  void clear_waypoints(const std::shared_ptr<ClearWaypoints::Request>,
                       std::shared_ptr<ClearWaypoints::Response> response)
  {
    response->cleared_count = static_cast<uint32_t>(queue_.size() + (active_goal_ ? 1 : 0));
    queue_.clear();
    active_goal_.reset();
    awaiting_execution_ = false;
    execution_observed_ = false;
    latest_ego_command_.reset();
    if (state_ == State::MOVE) {
      capture_hold();
      set_state(State::IDLE);
    }
    publish_queue_status();
    response->success = true;
    response->message = "cleared " + std::to_string(response->cleared_count) + " waypoint(s)";
  }

  void takeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ != State::INIT || takeoff_requested_) {
      response->success = false;
      response->message = "takeoff is accepted only while idle on the ground";
      return;
    }
    takeoff_requested_ = true;
    state_entered_ = now();
    response->success = true;
    response->message = "takeoff accepted";
  }

  void land(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::INIT || state_ == State::LAND) {
      response->success = false;
      response->message = "landing is unavailable in the current state";
      return;
    }
    queue_.clear();
    active_goal_.reset();
    awaiting_execution_ = false;
    execution_observed_ = false;
    latest_ego_command_.reset();
    publish_queue_status();
    land_command_sent_ = false;
    set_state(State::LAND);
    response->success = true;
    response->message = "native PX4 landing accepted";
  }

  void complete_active_goal()
  {
    if (active_goal_) {
      capture_hold();
      hold_yaw_ = static_cast<float>(offboard_fsm::frame::enu_yaw_to_ned(yaw_from_pose(*active_goal_)));
      RCLCPP_INFO(get_logger(), "EGO completed goal");
    }
    active_goal_.reset();
    awaiting_execution_ = false;
    execution_observed_ = false;
    latest_ego_command_.reset();
    publish_queue_status();
    set_state(State::IDLE);
  }

  void abort_route(const std::string &reason)
  {
    RCLCPP_ERROR(get_logger(), "Navigation aborted: %s", reason.c_str());
    queue_.clear();
    active_goal_.reset();
    awaiting_execution_ = false;
    execution_observed_ = false;
    latest_ego_command_.reset();
    capture_hold();
    publish_queue_status();
    set_state(State::IDLE);
  }

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode message;
    message.position = true;
    message.velocity = true;
    message.acceleration = true;
    message.timestamp = now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(message);
  }

  void publish_setpoint(float x, float y, float z, float vx, float vy, float vz,
                        float yaw, float yaw_rate = 0.0f,
                        float ax = std::numeric_limits<float>::quiet_NaN(),
                        float ay = std::numeric_limits<float>::quiet_NaN(),
                        float az = std::numeric_limits<float>::quiet_NaN())
  {
    px4_msgs::msg::TrajectorySetpoint message;
    message.position = {x, y, z};
    message.velocity = {vx, vy, vz};
    message.acceleration = {ax, ay, az};
    message.jerk = {NAN, NAN, NAN};
    message.yaw = yaw;
    message.yawspeed = yaw_rate;
    message.timestamp = now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(message);
  }

  void publish_hold()
  {
    publish_setpoint(hold_x_, hold_y_, hold_z_, 0.0f, 0.0f, 0.0f, hold_yaw_);
  }

  void send_vehicle_command(uint16_t command, float parameter1 = 0.0f, float parameter2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand message;
    message.command = command;
    message.param1 = parameter1;
    message.param2 = parameter2;
    message.target_system = 1;
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.from_external = true;
    message.timestamp = now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(message);
  }

  void publish_queue_status()
  {
    nav_msgs::msg::Path status;
    status.header.stamp = now();
    status.header.frame_id = frame_id_;
    if (active_goal_) {
      status.poses.push_back(*active_goal_);
    }
    for (const auto &waypoint : queue_) {
      status.poses.push_back(waypoint);
    }
    queue_status_pub_->publish(status);
  }

  void ensure_hold()
  {
    if (!have_hold_) {
      capture_hold();
    }
  }

  void capture_hold()
  {
    if (!local_position_) {
      return;
    }
    hold_x_ = local_position_->x;
    hold_y_ = local_position_->y;
    hold_z_ = local_position_->z;
    if (std::isfinite(local_position_->heading)) {
      hold_yaw_ = local_position_->heading;
    }
    have_hold_ = true;
  }

  bool has_valid_position() const
  {
    return local_position_ && local_position_->xy_valid && local_position_->z_valid &&
      std::isfinite(local_position_->x) && std::isfinite(local_position_->y) &&
      std::isfinite(local_position_->z);
  }

  bool is_armed() const
  {
    return vehicle_status_ &&
      vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_landed() const
  {
    return land_detected_ && land_detected_->landed;
  }

  bool valid_waypoint(const PoseStamped &waypoint) const
  {
    return std::isfinite(waypoint.pose.position.x) && std::isfinite(waypoint.pose.position.y) &&
      std::isfinite(waypoint.pose.position.z);
  }

  static double yaw_from_pose(const PoseStamped &pose)
  {
    const auto &q = pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm < 1e-8) {
      return 0.0;
    }
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double state_elapsed() const { return (now() - state_entered_).seconds(); }
  double time_since(const rclcpp::Time &time) const { return (now() - time).seconds(); }

  void set_state(State state)
  {
    if (state_ == state) {
      return;
    }
    RCLCPP_INFO(get_logger(), "State: %s -> %s", state_name(state_).c_str(), state_name(state).c_str());
    state_ = state;
    state_entered_ = now();
  }

  static std::string state_name(State state)
  {
    switch (state) {
      case State::INIT: return "INIT";
      case State::TAKEOFF: return "TAKEOFF";
      case State::IDLE: return "IDLE";
      case State::MOVE: return "MOVE";
      case State::LAND: return "LAND";
    }
    return "UNKNOWN";
  }

  State state_{State::INIT};
  std::string frame_id_;
  double update_rate_hz_{50.0};
  double warmup_sec_{1.0};
  double takeoff_height_m_{5.0};
  double takeoff_speed_mps_{0.6};
  double takeoff_tolerance_m_{0.3};
  double command_timeout_sec_{0.5};
  double goal_accept_timeout_sec_{8.0};
  double arm_retry_sec_{1.0};
  std::string local_position_topic_, vehicle_status_topic_, land_detected_topic_;
  std::string ego_goal_topic_, ego_state_topic_, ego_command_topic_;
  std::string queue_service_, clear_service_, takeoff_service_, land_service_, queue_status_topic_;
  std::string offboard_mode_topic_, trajectory_setpoint_topic_, vehicle_command_topic_;
  std::deque<PoseStamped> queue_;
  std::optional<PoseStamped> active_goal_;
  std::shared_ptr<px4_msgs::msg::VehicleLocalPosition> local_position_;
  std::shared_ptr<px4_msgs::msg::VehicleStatus> vehicle_status_;
  std::shared_ptr<px4_msgs::msg::VehicleLandDetected> land_detected_;
  std::shared_ptr<quadrotor_msgs::msg::PositionCommand> latest_ego_command_;
  std::string ego_state_;
  bool takeoff_requested_{false}, takeoff_target_set_{false}, have_hold_{false};
  bool awaiting_execution_{false}, execution_observed_{false}, land_command_sent_{false};
  float hold_x_{0.0f}, hold_y_{0.0f}, hold_z_{0.0f}, hold_yaw_{0.0f};
  float takeoff_x_{0.0f}, takeoff_y_{0.0f}, takeoff_z_{0.0f};
  rclcpp::Time state_entered_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_ego_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vehicle_command_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ego_state_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_command_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr ego_goal_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr queue_status_pub_;
  rclcpp::Service<QueueWaypoints>::SharedPtr queue_service_server_;
  rclcpp::Service<ClearWaypoints>::SharedPtr clear_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_server_;
};
}  // namespace

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
