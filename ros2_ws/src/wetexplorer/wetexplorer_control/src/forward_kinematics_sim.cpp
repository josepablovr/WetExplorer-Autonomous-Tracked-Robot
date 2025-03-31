#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <string>
#include <unordered_map>

class ForwardKinematics : public rclcpp::Node
{
public:
    ForwardKinematics()
    : Node("forward_kinematics_node"),
      tracks_separation_(0.6108),
      gear_ratio_(1.0),
      radius_sprocket_(0.075),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      last_time_(this->now())
    {
        using std::placeholders::_1;

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ForwardKinematics::jointStateCallback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/forward_kinematics", 10);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received JointState message with %zu joints", msg->name.size());

        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string joint_name = msg->name[i];
            double velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;

            //RCLCPP_INFO(this->get_logger(), "Joint: '%s', Velocity: %.6f", joint_name.c_str(), velocity);
        }

        double omega_L = 0.0;
        double omega_R = 0.0;
        bool left_found = false;
        bool right_found = false;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i >= msg->velocity.size()) continue;

            if (msg->name[i] == "left_wheel_joint") {
                omega_L = msg->velocity[i];
                left_found = true;
            } else if (msg->name[i] == "right_wheel_joint") {
                omega_R = msg->velocity[i];
                right_found = true;
            }
        }

        if (!left_found || !right_found) {
            //RCLCPP_WARN(this->get_logger(), "Missing joint velocity data for left or right wheel joint.");
            return;
        }

        // Convert rad/s -> linear m/s (accounting for gear ratio & sprocket radius)
        double v_L = (omega_L * radius_sprocket_) / gear_ratio_;
        double v_R = (omega_R * radius_sprocket_) / gear_ratio_;

        // Forward kinematics
        double V_x = (v_L + v_R) / 2.0;
        double theta_dot = (v_R - v_L) / tracks_separation_;

        publishOdometry(V_x, theta_dot);
    }

    void publishOdometry(double linear_velocity, double angular_velocity)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        theta_ += angular_velocity * dt;
        x_ += linear_velocity * dt * std::cos(theta_);
        y_ += linear_velocity * dt * std::sin(theta_);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;


        // Set covariance values
        for (int i = 0; i < 36; ++i) {
            if (i == 0) {
              
                if (abs(linear_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.000001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.000001; 
                }
            } else if (i == 7) {
                
                if (abs(linear_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.0001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.0001; 
                }
                
            } else if (i == 35) {
                if (abs(angular_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.00001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.5;
                }
                 
            } else if (i == 14 || i == 21 || i == 28) {
                odom_msg.twist.covariance[i] = 99999.0;
            } else {
                odom_msg.twist.covariance[i] = 0.0;
            }
        }

        odom_pub_->publish(odom_msg);
        last_time_ = current_time;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double tracks_separation_;
    double gear_ratio_;
    double radius_sprocket_;

    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}
