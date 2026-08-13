#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace offboard
{

/**
 * @brief Accept user waypoints and forward them to the offboard waypoint buffer.
 *
 * Subscribes to /waypoint_pose (geometry_msgs/PoseStamped) — the separate
 * waypoint topic, e.g. the RViz "2D Goal Pose" tool re-targeted to
 * /waypoint_pose — and forwards each waypoint to /waypoint_buffer, where the
 * offboard node stores it and flies it. The offboard node publishes the waypoint
 * buffer state regularly on /waypoint_markers for RViz.
 *
 * The original /goal_pose topic is left untouched — it is reserved for the
 * offboard node to hand goals to SUPER (one at a time).
 */
class GoalMarkerNode : public rclcpp::Node
{
public:
    explicit GoalMarkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("goal_marker", options)
    {
        waypoint_topic_ = declare_parameter("waypoint_topic", waypoint_topic_);
        waypoint_buffer_topic_ = declare_parameter("waypoint_buffer_topic", waypoint_buffer_topic_);

        // Waypoint buffer publisher (reliable so the offboard node never drops one).
        buffer_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            waypoint_buffer_topic_, rclcpp::QoS(10).reliable());

        // Waypoints are published by the user (RViz "2D Goal Pose" re-targeted to
        // /waypoint_pose); it publishes reliable/volatile — accept it.
        waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            waypoint_topic_, rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile(),
            std::bind(&GoalMarkerNode::waypointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Goal marker node started. %s -> %s",
                    waypoint_topic_.c_str(), waypoint_buffer_topic_.c_str());
    }

private:
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Waypoint received: (%.2f, %.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // Forward the waypoint to the offboard node's waypoint buffer.
        buffer_pub_->publish(*msg);
    }

    std::string waypoint_topic_{"/waypoint_pose"};
    std::string waypoint_buffer_topic_{"/waypoint_buffer"};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr buffer_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
};

}  // namespace offboard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::GoalMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
