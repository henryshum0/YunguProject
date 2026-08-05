#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace offboard
{

/**
 * @brief Republish the RViz "2D Goal Pose" as visualization markers.
 *
 * Subscribes to /goal_pose (geometry_msgs/PoseStamped) and publishes a green
 * sphere at the goal plus a cyan arrow showing the requested yaw to
 * /goal_marker. Transient-local so a late-starting RViz still sees the last
 * goal.
 */
class GoalMarkerNode : public rclcpp::Node
{
public:
    explicit GoalMarkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("goal_marker", options)
    {
        goal_topic_ = declare_parameter("goal_topic", goal_topic_);
        goal_marker_topic_ = declare_parameter("goal_marker_topic", goal_marker_topic_);

        // Transient-local so a late-starting RViz still sees the latest goal marker.
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            goal_marker_topic_, rclcpp::QoS(1).transient_local());

        // RViz "2D Goal Pose" publishes reliable/volatile; accept it.
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile(),
            std::bind(&GoalMarkerNode::goalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Goal marker node started. %s -> %s",
                    goal_topic_.c_str(), goal_marker_topic_.c_str());
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Goal received: (%.2f, %.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // Green sphere at the clicked goal position.
        visualization_msgs::msg::Marker sphere;
        sphere.header = msg->header;
        sphere.ns = "goal";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose = msg->pose;
        sphere.scale.x = 0.3;
        sphere.scale.y = 0.3;
        sphere.scale.z = 0.3;
        sphere.color.r = 0.0f;
        sphere.color.g = 1.0f;
        sphere.color.b = 0.0f;
        sphere.color.a = 1.0f;
        marker_pub_->publish(sphere);

        // Cyan arrow showing the goal yaw orientation.
        visualization_msgs::msg::Marker arrow;
        arrow.header = msg->header;
        arrow.ns = "goal";
        arrow.id = 1;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = msg->pose;
        arrow.scale.x = 0.6;
        arrow.scale.y = 0.12;
        arrow.scale.z = 0.12;
        arrow.color.r = 0.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;
        marker_pub_->publish(arrow);
    }

    std::string goal_topic_{"/goal_pose"};
    std::string goal_marker_topic_{"/goal_marker"};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace offboard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::GoalMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
