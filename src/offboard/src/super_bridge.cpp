#include <rclcpp/rclcpp.hpp>

#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "offboard/frame_conversion.hpp"

namespace offboard
{

/**
 * @brief Transform the fused lidar cloud into the world (ENU) frame using
 * PX4's odometry.
 *
 * The input cloud (default /swan_gamma_v2/scan/points_fused) is the output of
 * lidar_merge, i.e. the two side LiDARs already transformed into base_link
 * (lidar_merge applied the mounting extrinsics). SUPER's ROG-Map treats the
 * input cloud as world-frame coordinates, so each base_link point is rotated
 * by the drone's ENU attitude and translated by its ENU position:
 *
 *   p_world = R_enu * p_base + t_enu
 *
 * No lidar_link->base_link offset is applied: the fused cloud is already in
 * base_link (lidar_offset_* defaults to 0; kept only for a raw lidar_link-
 * frame input).
 *
 * The ENU pose is also republished as nav_msgs/Odometry (default
 * /lidar_slam/odom) so SUPER's ray origin matches the registered cloud.
 */
class SuperBridge : public rclcpp::Node
{
public:
    explicit SuperBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("super_bridge", options)
    {
        cloud_in_topic_ = declare_parameter("cloud_in_topic", cloud_in_topic_);
        odom_topic_ = declare_parameter("odom_topic", odom_topic_);
        cloud_out_topic_ = declare_parameter("cloud_out_topic", cloud_out_topic_);
        odom_out_topic_ = declare_parameter("odom_out_topic", odom_out_topic_);
        world_frame_ = declare_parameter("world_frame", world_frame_);
        base_frame_ = declare_parameter("base_frame", base_frame_);
        lidar_offset_x_ = declare_parameter("lidar_offset_x", lidar_offset_x_);
        lidar_offset_y_ = declare_parameter("lidar_offset_y", lidar_offset_y_);
        lidar_offset_z_ = declare_parameter("lidar_offset_z", lidar_offset_z_);

        auto qos = rclcpp::QoS(5).best_effort().durability_volatile();

        cloud_in_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic_, qos,
            std::bind(&SuperBridge::cloudCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            odom_topic_, qos,
            std::bind(&SuperBridge::odomCallback, this, std::placeholders::_1));

        cloud_out_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic_, qos);
        odom_out_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out_topic_, qos);

        RCLCPP_INFO(get_logger(),
                    "Super bridge: %s (lidar_link) --px4 odom %s--> %s (%s) + %s",
                    cloud_in_topic_.c_str(), odom_topic_.c_str(),
                    cloud_out_topic_.c_str(), world_frame_.c_str(),
                    odom_out_topic_.c_str());
    }

private:
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // PX4 vehicle_odometry is NED; convert to ENU so the output cloud and
        // odometry match the ENU world used by SUPER.
        double pxe, pye, pze, qew, qex, qey, qez;
        frame::nedToEnu(msg->position[0], msg->position[1], msg->position[2],
                        msg->q[0], msg->q[1], msg->q[2], msg->q[3],
                        pxe, pye, pze, qew, qex, qey, qez);

        pose_valid_ = true;
        px_ = pxe;
        py_ = pye;
        pz_ = pze;
        qw_ = qew;
        qx_ = qex;
        qy_ = qey;
        qz_ = qez;

        // Republish as a world-frame (ENU) nav_msgs::Odometry for SUPER.
        nav_msgs::msg::Odometry out;
        out.header.stamp = now();
        out.header.frame_id = world_frame_;
        out.child_frame_id = base_frame_;
        out.pose.pose.position.x = pxe;
        out.pose.pose.position.y = pye;
        out.pose.pose.position.z = pze;
        out.pose.pose.orientation.w = qew;
        out.pose.pose.orientation.x = qex;
        out.pose.pose.orientation.y = qey;
        out.pose.pose.orientation.z = qez;
        // Velocity NED -> ENU (same swap as position).
        out.twist.twist.linear.x = msg->velocity[1];
        out.twist.twist.linear.y = msg->velocity[0];
        out.twist.twist.linear.z = -msg->velocity[2];
        odom_out_pub_->publish(out);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!pose_valid_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No PX4 odom yet, skipping cloud frame");
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

            // The fused cloud is already in base_link (lidar_merge applied the
            // side-lidar extrinsics) — no lidar_link offset here. base_link →
            // world via the drone's ENU attitude + position.
            const double bx = lx + lidar_offset_x_;
            const double by = ly + lidar_offset_y_;
            const double bz = lz + lidar_offset_z_;

            double wx, wy, wz;
            frame::rotateByQuat(qw_, qx_, qy_, qz_, bx, by, bz, wx, wy, wz);
            const float fwx = static_cast<float>(wx + px_);
            const float fwy = static_cast<float>(wy + py_);
            const float fwz = static_cast<float>(wz + pz_);

            std::memcpy(p + x_off, &fwx, sizeof(float));
            std::memcpy(p + y_off, &fwy, sizeof(float));
            std::memcpy(p + z_off, &fwz, sizeof(float));
        }

        cloud_out_pub_->publish(*out);
    }

    std::string cloud_in_topic_{"/swan_gamma_v2/scan/points"};
    std::string odom_topic_{"/fmu/out/vehicle_odometry"};
    std::string cloud_out_topic_{"/cloud_registered"};
    std::string odom_out_topic_{"/lidar_slam/odom"};
    std::string world_frame_{"world"};
    std::string base_frame_{"base_link"};
    double lidar_offset_x_{0.0};
    double lidar_offset_y_{0.0};
    // Input cloud is the fused one in base_link -> no lidar_link offset. Was
    // 0.16 when super_bridge consumed a raw lidar_link-frame cloud; set it back
    // to 0.16 only if cloud_in_topic is pointed at such a cloud.
    double lidar_offset_z_{0.0};

    bool pose_valid_{false};
    double px_{0.0}, py_{0.0}, pz_{0.0};
    double qw_{1.0}, qx_{0.0}, qy_{0.0}, qz_{0.0};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_pub_;
};

}  // namespace offboard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::SuperBridge>());
    rclcpp::shutdown();
    return 0;
}
