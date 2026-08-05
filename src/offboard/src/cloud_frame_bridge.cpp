#include <rclcpp/rclcpp.hpp>

#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace offboard
{

// Rotate vector v by the unit quaternion q (Hamilton convention, w-first):
//   v' = q * v * q^-1
static void rotateByQuat(double qw, double qx, double qy, double qz,
                         double vx, double vy, double vz,
                         double &ox, double &oy, double &oz)
{
    // t = 2 * cross(q.xyz, v)
    const double tx = 2.0 * (qy * vz - qz * vy);
    const double ty = 2.0 * (qz * vx - qx * vz);
    const double tz = 2.0 * (qx * vy - qy * vx);
    // v' = v + qw * t + cross(q.xyz, t)
    const double cx = qy * tz - qz * ty;
    const double cy = qz * tx - qx * tz;
    const double cz = qx * ty - qy * tx;
    ox = vx + qw * tx + cx;
    oy = vy + qw * ty + cy;
    oz = vz + qw * tz + cz;
}

/**
 * @brief Transform the raw gz-bridged lidar cloud into the world frame.
 *
 * The gz lidar (/x500_lidar/scan/points) publishes points in the lidar_link
 * (sensor) frame, but SUPER's ROG-Map treats the input cloud as world-frame
 * coordinates. This node registers the cloud using the world-frame odometry
 * (/odom) plus the static lidar_link -> base_link offset:
 *
 *   p_world = R_odom * (p_lidar + offset) + t_odom
 *
 * and republishes it (default /cloud_registered) with frame_id "world", plus
 * a pass-through odometry topic (default /lidar_slam/odom), matching the
 * topics used in super_planner/config/static_dense.yaml.
 */
class CloudFrameBridge : public rclcpp::Node
{
public:
    explicit CloudFrameBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("cloud_frame_bridge", options)
    {
        cloud_in_topic_ = declare_parameter("cloud_in_topic", cloud_in_topic_);
        odom_topic_ = declare_parameter("odom_topic", odom_topic_);
        cloud_out_topic_ = declare_parameter("cloud_out_topic", cloud_out_topic_);
        odom_out_topic_ = declare_parameter("odom_out_topic", odom_out_topic_);
        world_frame_ = declare_parameter("world_frame", world_frame_);
        lidar_offset_x_ = declare_parameter("lidar_offset_x", lidar_offset_x_);
        lidar_offset_y_ = declare_parameter("lidar_offset_y", lidar_offset_y_);
        lidar_offset_z_ = declare_parameter("lidar_offset_z", lidar_offset_z_);

        auto qos = rclcpp::QoS(5).best_effort().durability_volatile();

        cloud_in_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic_, qos,
            std::bind(&CloudFrameBridge::cloudCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos,
            std::bind(&CloudFrameBridge::odomCallback, this, std::placeholders::_1));

        cloud_out_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic_, qos);
        odom_out_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out_topic_, qos);

        RCLCPP_INFO(get_logger(),
                    "Cloud frame bridge: %s (lidar_link) --odom %s--> %s (%s) + %s",
                    cloud_in_topic_.c_str(), odom_topic_.c_str(),
                    cloud_out_topic_.c_str(), world_frame_.c_str(),
                    odom_out_topic_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_valid_ = true;
        px_ = msg->pose.pose.position.x;
        py_ = msg->pose.pose.position.y;
        pz_ = msg->pose.pose.position.z;
        qw_ = msg->pose.pose.orientation.w;
        qx_ = msg->pose.pose.orientation.x;
        qy_ = msg->pose.pose.orientation.y;
        qz_ = msg->pose.pose.orientation.z;
        // Pass through under the output odom topic (already world frame).
        odom_out_pub_->publish(*msg);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!pose_valid_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No odom pose yet, skipping cloud frame");
            return;
        }

        int x_off = -1, y_off = -1, z_off = -1;
        for (const auto &f : msg->fields) {
            if (f.name == "x") {
                x_off = static_cast<int>(f.offset);
            } else if (f.name == "y") {
                y_off = static_cast<int>(f.offset);
            } else if (f.name == "z") {
                z_off = static_cast<int>(f.offset);
            }
        }
        if (x_off < 0 || y_off < 0 || z_off < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Cloud has no float32 x/y/z fields, skipping");
            return;
        }

        auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
        out->header.frame_id = world_frame_;

        const size_t step = out->point_step;
        const size_t n = out->width * out->height;
        for (size_t i = 0; i < n; ++i) {
            uint8_t *p = out->data.data() + i * step;
            float lx, ly, lz;
            std::memcpy(&lx, p + x_off, sizeof(float));
            std::memcpy(&ly, p + y_off, sizeof(float));
            std::memcpy(&lz, p + z_off, sizeof(float));

            // lidar_link -> base_link (static offset), then base_link -> world
            const double bx = lx + lidar_offset_x_;
            const double by = ly + lidar_offset_y_;
            const double bz = lz + lidar_offset_z_;

            double wx, wy, wz;
            rotateByQuat(qw_, qx_, qy_, qz_, bx, by, bz, wx, wy, wz);
            const float fwx = static_cast<float>(wx + px_);
            const float fwy = static_cast<float>(wy + py_);
            const float fwz = static_cast<float>(wz + pz_);

            std::memcpy(p + x_off, &fwx, sizeof(float));
            std::memcpy(p + y_off, &fwy, sizeof(float));
            std::memcpy(p + z_off, &fwz, sizeof(float));
        }

        cloud_out_pub_->publish(*out);
    }

    std::string cloud_in_topic_{"/x500_lidar/scan/points"};
    std::string odom_topic_{"/odom"};
    std::string cloud_out_topic_{"/cloud_registered"};
    std::string odom_out_topic_{"/lidar_slam/odom"};
    std::string world_frame_{"world"};
    double lidar_offset_x_{0.0};
    double lidar_offset_y_{0.0};
    double lidar_offset_z_{0.16};  // lidar_link is 0.16 m above base_link

    bool pose_valid_{false};
    double px_{0.0}, py_{0.0}, pz_{0.0};
    double qw_{1.0}, qx_{0.0}, qy_{0.0}, qz_{0.0};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_pub_;
};

}  // namespace offboard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::CloudFrameBridge>());
    rclcpp::shutdown();
    return 0;
}
