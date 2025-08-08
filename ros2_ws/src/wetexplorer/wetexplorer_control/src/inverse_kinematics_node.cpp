#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <cmath>
#include <vector>
#include <algorithm>

class RobotControl : public rclcpp::Node
{
public:
    RobotControl()
    : Node("robot_control_node"),
      radius_sprocket_(0.076678), //0.083
      track_separation_(0.6103),
      b_(0.6108),
      gear_ratio_(30.0),
      vel_max_rpm_(1500.0)
    {
        using std::placeholders::_1;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/WetExplorer/cmd_vel", 10, std::bind(&RobotControl::cmdVelCallback, this, _1));

        motor_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robo/cmd_vel", 10);
        angular_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/angular_velocity_cmd", 10);
        b_ *=1.19;
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
    {
        double V_x = vel->linear.x;
        double theta_dot = vel->angular.z;

        double omega_L, omega_R;
        inverseKinematics(V_x, theta_dot, radius_sprocket_, omega_L, omega_R);

        double omega_L_rpm = (omega_L * 60.0) / (2 * M_PI);
        double omega_R_rpm = (omega_R * 60.0) / (2 * M_PI);

        omega_L_rpm *= gear_ratio_;
        omega_R_rpm *= gear_ratio_;

        // Publish angular velocities
        std_msgs::msg::Float32MultiArray angular_velocity_cmd;
        angular_velocity_cmd.data.push_back(omega_R_rpm);
        angular_velocity_cmd.data.push_back(omega_L_rpm);
        angular_velocity_pub_->publish(angular_velocity_cmd);

        // Convert to percentage of max velocity
        double omega_L_percent = (omega_L_rpm / vel_max_rpm_) * 1000.0;
        double omega_R_percent = (omega_R_rpm / vel_max_rpm_) * 1000.0;

        omega_L_percent = std::clamp(omega_L_percent, -1000.0, 1000.0);
        omega_R_percent = std::clamp(omega_R_percent, -1000.0, 1000.0);

        int omega_L_int = static_cast<int>(std::round(omega_L_percent));
        int omega_R_int = static_cast<int>(std::round(omega_R_percent));

        geometry_msgs::msg::Twist motor_cmd;
        motor_cmd.linear.x = omega_R_int;
        motor_cmd.angular.z = omega_L_int;

        motor_cmd_pub_->publish(motor_cmd);

        RCLCPP_INFO(this->get_logger(), "Published motor commands: R = %.2f, L = %.2f", omega_R, omega_L);
    }

    void inverseKinematics(double V_x, double theta_dot, double r, double& omega_L, double& omega_R)
    {
        omega_L = (V_x / r) - (b_ * theta_dot / (2 * r));
        omega_R = (V_x / r) + (b_ * theta_dot / (2 * r));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angular_velocity_pub_;

    // Constants
    double radius_sprocket_;
    const double track_separation_;
    double b_;
    double gear_ratio_;
    double vel_max_rpm_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControl>());
    rclcpp::shutdown();
    return 0;
}
