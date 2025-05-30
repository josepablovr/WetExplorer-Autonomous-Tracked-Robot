#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class GroundTruthOdometry : public rclcpp::Node
{
public:
    GroundTruthOdometry()
    : Node("ground_truth_odometry_node"),
      got_first_fix_(false),
      got_first_imu_(false),
      heading_(0.0),
      heading_rate_z_(0.0)
    {
        using std::placeholders::_1;

        gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/gps_ref", 10, std::bind(&GroundTruthOdometry::gpsCallback, this, _1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_heading/data", 10, std::bind(&GroundTruthOdometry::imuCallback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/ground_truth", 10);
    }

private:
    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_gps_pose_ = msg->pose;
        gps_time_ = this->now();
        got_first_fix_ = true;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto now = this->now();

        // Extract yaw (heading) from IMU quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        heading_ = yaw;
        heading_rate_z_ = msg->angular_velocity.z;

        // If we haven’t received GPS yet, publish zeroed odometry
        if (!got_first_fix_) {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = 0.0;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q_zero;
            q_zero.setRPY(0.0, 0.0, heading_);
            //odom_msg.pose.pose.orientation = tf2::toMsg(q_zero);

            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;

            odom_pub_->publish(odom_msg);
            return;
        }

        // --- Compute velocities from GPS ---
        double x = latest_gps_pose_.pose.position.x;
        double y = latest_gps_pose_.pose.position.y;

        if (!got_first_imu_) {
            last_time_ = now;
            last_x_ = x;
            last_y_ = y;
            got_first_imu_ = true;
            return;
        }

        double dt = (now - last_time_).seconds();
        if (dt < 1e-6) return;

        double dx = x - last_x_;
        double dy = y - last_y_;

        // Global velocities (odom frame)
        double vx_odom = dx / dt;
        double vy_odom = dy / dt;

        // Transform to base_link frame using heading
        double cos_h = std::cos(-heading_);
        double sin_h = std::sin(-heading_);
        double vx_bl = cos_h * vx_odom - sin_h * vy_odom;
        double vy_bl = sin_h * vx_odom + cos_h * vy_odom;

        // Fill odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose = latest_gps_pose_;
        odom_msg.twist.twist.linear.x = vx_bl;
        odom_msg.twist.twist.linear.y = vy_bl;
        odom_msg.twist.twist.angular.z = heading_rate_z_;

        odom_pub_->publish(odom_msg);

        // Store for next cycle
        last_time_ = now;
        last_x_ = x;
        last_y_ = y;
    }

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // State
    geometry_msgs::msg::PoseWithCovariance latest_gps_pose_;
    rclcpp::Time gps_time_;
    rclcpp::Time last_time_;
    double last_x_;
    double last_y_;
    bool got_first_fix_;
    bool got_first_imu_;

    double heading_;
    double heading_rate_z_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthOdometry>());
    rclcpp::shutdown();
    return 0;
}
