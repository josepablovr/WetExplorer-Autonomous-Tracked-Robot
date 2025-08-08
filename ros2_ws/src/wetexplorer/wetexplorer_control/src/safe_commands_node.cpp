#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class SafeCommands : public rclcpp::Node
{
public:
    SafeCommands()
    : Node("safe_commands"),
      last_linear_velocity_(0.0),
      last_angular_velocity_(0.0),
      last_time_(this->now())
    {
        using std::placeholders::_1;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_out", 10, std::bind(&SafeCommands::cmdVelCallback, this, _1));

        safe_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/WetExplorer/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
    rclcpp::Time last_time_;
    double last_linear_velocity_;
    double last_angular_velocity_;
    

    const double max_linear_velocity_ = 0.5;
    const double max_linear_acceleration_ = 0.5;
    const double max_linear_deceleration_ = 2.5;
    const double max_angular_velocity_ = 1.0;
    const double max_angular_acceleration_ = 0.5;
    const double max_angular_deceleration_ = 1.0;
    const double velocity_threshold_ = 0.01;
   

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist safe_cmd;

        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        // Avoid very small or zero dt
        if (dt < 1e-6)
            dt = 1e-6;
        else if (dt > 0.5){
            RCLCPP_WARN(this->get_logger(), "Large dt (%.3f s), resetting velocity history", dt);
            last_linear_velocity_ = msg->linear.x;
            last_angular_velocity_ = msg->angular.z;
            dt = 0.1; // fallback to a default reasonable value
        }

        double linear_acceleration = (linear_velocity - last_linear_velocity_) / dt;
        double angular_acceleration = (angular_velocity - last_angular_velocity_) / dt;

        // --- Linear velocity limiting ---
        if (std::fabs(linear_velocity) > velocity_threshold_)
        {
            // Clamp velocity
            linear_velocity = std::copysign(
                std::min(std::fabs(linear_velocity), max_linear_velocity_), linear_velocity);

            // Clamp acceleration/deceleration
            if (linear_acceleration > 0 && std::fabs(linear_acceleration) > max_linear_acceleration_)
            {
                linear_velocity = last_linear_velocity_ + std::copysign(max_linear_acceleration_ * dt, linear_velocity - last_linear_velocity_);
            }
            else if (linear_acceleration < 0 && std::fabs(linear_acceleration) > max_linear_deceleration_)
            {
                linear_velocity = last_linear_velocity_ + std::copysign(max_linear_deceleration_ * dt, linear_velocity - last_linear_velocity_);
            }
        }

        // --- Angular velocity limiting ---
        if (std::fabs(angular_velocity) > velocity_threshold_)
        {
            angular_velocity = std::copysign(
                std::min(std::fabs(angular_velocity), max_angular_velocity_), angular_velocity);

            if (angular_acceleration > 0 && std::fabs(angular_acceleration) > max_angular_acceleration_)
            {
                angular_velocity = last_angular_velocity_ + std::copysign(max_angular_acceleration_ * dt, angular_velocity - last_angular_velocity_);
            }
            else if (angular_acceleration < 0 && std::fabs(angular_acceleration) > max_angular_deceleration_)
            {
                angular_velocity = last_angular_velocity_ + std::copysign(max_angular_deceleration_ * dt, angular_velocity - last_angular_velocity_);
            }
        }

        safe_cmd.linear.x = linear_velocity;
        safe_cmd.angular.z = angular_velocity;

        safe_cmd_vel_pub_->publish(safe_cmd);

        last_linear_velocity_ = linear_velocity;
        last_angular_velocity_ = angular_velocity;
    }
};

int main(int argc, char **argv)
{
   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeCommands>());
    rclcpp::shutdown();
    return 0;
}
