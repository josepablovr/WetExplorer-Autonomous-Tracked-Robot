// ROS 2 C++ version of the Python odometry_tcp node
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>

class OdometryTcpPublisher : public rclcpp::Node {
public:
    OdometryTcpPublisher() : Node("odometry_tcp"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odometry/tcp", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/local", 10,
            std::bind(&OdometryTcpPublisher::odometry_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    
    tf2_ros::TransformListener tf_listener_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odometry callback triggered");
        try {
            auto transform_odom_base_link = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            auto transform_base_link_chamber = tf_buffer_.lookupTransform("base_link", "chamber_link", tf2::TimePointZero);
            RCLCPP_DEBUG(this->get_logger(), "TF lookup successful for both transforms");

            Eigen::Vector3d T_odom_base_link(
                transform_odom_base_link.transform.translation.x,
                transform_odom_base_link.transform.translation.y,
                transform_odom_base_link.transform.translation.z);

            tf2::Quaternion q_odom_base;
            tf2::fromMsg(transform_odom_base_link.transform.rotation, q_odom_base);
            tf2::Matrix3x3 R_odom_base(q_odom_base);

            Eigen::Matrix3d R_odom_base_link;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R_odom_base_link(i, j) = R_odom_base[i][j];

            Eigen::Vector3d T_base_link_chamber(
                transform_base_link_chamber.transform.translation.x,
                transform_base_link_chamber.transform.translation.y,
                transform_base_link_chamber.transform.translation.z);

            tf2::Quaternion q_base_chamber;
            tf2::fromMsg(transform_base_link_chamber.transform.rotation, q_base_chamber);
            tf2::Matrix3x3 R_base_chamber(q_base_chamber);

            Eigen::Matrix3d R_base_link_chamber;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R_base_link_chamber(i, j) = R_base_chamber[i][j];

            // Compute combined transform: odom -> chamber_link
            auto [R_odom_chamber, T_odom_chamber] = compute_transform(
                R_odom_base_link, T_odom_base_link,
                R_base_link_chamber, T_base_link_chamber);

            double theta_base = std::atan2(R_odom_base_link(1, 0), R_odom_base_link(0, 0));

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = "odom";
            pose_msg.pose.position.x = T_odom_chamber(0);
            pose_msg.pose.position.y = T_odom_chamber(1);
            pose_msg.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_base);
            pose_msg.pose.orientation = tf2::toMsg(q);

            pose_pub_->publish(pose_msg);
            RCLCPP_DEBUG(this->get_logger(), "Topic Published");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    std::pair<Eigen::Matrix3d, Eigen::Vector3d> compute_transform(
        const Eigen::Matrix3d &R1, const Eigen::Vector3d &T1,
        const Eigen::Matrix3d &R2, const Eigen::Vector3d &T2) {
        Eigen::Matrix3d R_combined = R1 * R2;
        Eigen::Vector3d T_combined = R1 * T2 + T1;
        return {R_combined, T_combined};
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTcpPublisher>());
    rclcpp::shutdown();
    return 0;
}