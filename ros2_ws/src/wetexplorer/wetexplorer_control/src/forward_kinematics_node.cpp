#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>

class ForwardKinematics : public rclcpp::Node
{
public:
    ForwardKinematics()
    : Node("forward_kinematics_node"),
      tracks_separation_(0.6108),
      gear_ratio_(30.0),
      radius_sprocket_(0.0754),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      last_time_(this->now())
    {
        using std::placeholders::_1;

        // Subscribe to /robo/sys
        velocity_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robo/sys", 10, std::bind(&ForwardKinematics::velocityCallback, this, _1));

        // Publish to /odometry/forward_kinematics
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/forward_kinematics", 10);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        pos_left_ = 0.0;
        pos_right_ = 0.0;
            
    }

private:
    // The callback that processes the incoming velocity messages
    void velocityCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse the incoming string
        std::string data = msg->data;
        std::vector<std::string> tokens;
        {
            std::stringstream ss(data);
            std::string token;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
        }
        // Safety check
        if (tokens.size() < 5) {
            RCLCPP_WARN(this->get_logger(), "Not enough tokens in velocity message.");
            return;
        }

        double omega_L_rpm = 0.0;
        double omega_R_rpm = 0.0;

        try {
            omega_L_rpm = std::stod(tokens[4]);
            omega_R_rpm = std::stod(tokens[3]);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Could not parse RPM values: %s", e.what());
            return;
        }

        // Convert RPM -> rad/s
        double omega_L = (omega_L_rpm * 2.0 * M_PI) / 60.0;
        double omega_R = (omega_R_rpm * 2.0 * M_PI) / 60.0;

        // Convert rad/s -> linear m/s (accounting for gear ratio & sprocket radius)
        double v_L = (omega_L * radius_sprocket_) / gear_ratio_;
        double v_R = (omega_R * radius_sprocket_) / gear_ratio_;

        // Forward kinematic equations
        double V_x = (v_L + v_R) / 2.0;
        double theta_dot = (v_R - v_L) / tracks_separation_;

        
        double dt = (this->now() - last_time_).seconds();
        pos_left_ += omega_L * dt;
        pos_right_ += omega_R * dt;

        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = this->now();
        js_msg.name = {
        "sprocket_left_joint", "sprocket_right_joint",
        "track_left_joint", "track_right_joint"
        };
        js_msg.position = {
        pos_left_, pos_right_,
        0.0, 0.0
        };

        joint_state_pub_->publish(js_msg);
        publishOdometry(V_x, theta_dot);

    }

    // Publish a nav_msgs/Odometry with updated pose
    void publishOdometry(double linear_velocity, double angular_velocity)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        // Update pose
        theta_ += angular_velocity * dt;
        x_ += linear_velocity * dt * std::cos(theta_);
        y_ += linear_velocity * dt * std::sin(theta_);

        // Populate Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Convert yaw -> Quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        // (Optional) set covariance values
        //  ...

        odom_pub_->publish(odom_msg);

        last_time_ = current_time;
    }

    // Subscriptions/Pubs
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr velocity_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    double pos_left_, pos_right_;


    // Robot parameters
    double tracks_separation_;
    double gear_ratio_;
    double radius_sprocket_;

    // Pose
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
