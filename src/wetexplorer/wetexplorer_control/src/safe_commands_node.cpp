#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class SafeCommands
{
public:
    SafeCommands()
    {
        // Initialize the subscriber and publisher
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel_out", 10, &SafeCommands::cmdVelCallback, this);
        safe_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/WetExplorer/cmd_vel", 10);

        // Initialize last velocities to zero
        last_linear_velocity_ = 0.0;
        last_angular_velocity_ = 0.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher safe_cmd_vel_pub_;

    double last_linear_velocity_;
    double last_angular_velocity_;

    const double max_linear_velocity_ = 0.5;            // m/s
    const double max_linear_acceleration_ = 0.15;        // m/s^2 (acceleration)r
    const double max_linear_deceleration_ = 0.25;        // m/s^2 (deceleration)
    const double max_angular_velocity_ = 1.0;           // rad/s
    const double max_angular_acceleration_ = 0.1;      // rad/s^2 (acceleration)
    const double max_angular_deceleration_ = 0.25;      // rad/s^2 (deceleration)
    const double velocity_threshold_ = 0.01;             // m/s, threshold below which no limits are applied

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        geometry_msgs::Twist safe_cmd;

        // Handle linear velocity and acceleration
        double linear_velocity = msg->linear.x;
        double linear_acceleration = (linear_velocity - last_linear_velocity_) / 0.1; // Assuming 10 Hz rate

        if (std::fabs(linear_velocity) > velocity_threshold_)
        {
            // Apply velocity limit
            if (std::fabs(linear_velocity) > max_linear_velocity_)
            {
                linear_velocity = std::copysign(max_linear_velocity_, linear_velocity);
            }

            // Apply appropriate acceleration or deceleration limit
            if (linear_acceleration > 0) // Accelerating
            {
                if (std::fabs(linear_acceleration) > max_linear_acceleration_)
                {
                    linear_velocity = last_linear_velocity_ + std::copysign(max_linear_acceleration_ * 0.1, linear_velocity - last_linear_velocity_);
                }
            }
            else if (linear_acceleration < 0) // Decelerating
            {
                if (std::fabs(linear_acceleration) > max_linear_deceleration_)
                {
                    linear_velocity = last_linear_velocity_ + std::copysign(max_linear_deceleration_ * 0.1, linear_velocity - last_linear_velocity_);
                }
            }
        }

        // Handle angular velocity and acceleration
        double angular_velocity = msg->angular.z;
        double angular_acceleration = (angular_velocity - last_angular_velocity_) / 0.1; // Assuming 10 Hz rate

        if (std::fabs(angular_velocity) > velocity_threshold_)
        {
            // Apply velocity limit
            if (std::fabs(angular_velocity) > max_angular_velocity_)
            {
                angular_velocity = std::copysign(max_angular_velocity_, angular_velocity);
            }

            // Apply appropriate acceleration or deceleration limit
            if (angular_acceleration > 0) // Accelerating
            {
                if (std::fabs(angular_acceleration) > max_angular_acceleration_)
                {
                    angular_velocity = last_angular_velocity_ + std::copysign(max_angular_acceleration_ * 0.1, angular_velocity - last_angular_velocity_);
                }
            }
            else if (angular_acceleration < 0) // Decelerating
            {
                if (std::fabs(angular_acceleration) > max_angular_deceleration_)
                {
                    angular_velocity = last_angular_velocity_ + std::copysign(max_angular_deceleration_ * 0.1, angular_velocity - last_angular_velocity_);
                }
            }
        }

        // Set the safe velocities
        safe_cmd.linear.x = linear_velocity;
        safe_cmd.angular.z = angular_velocity;

        // Publish the safe command
        safe_cmd_vel_pub_.publish(safe_cmd);

        // Update the last velocities
        last_linear_velocity_ = linear_velocity;
        last_angular_velocity_ = angular_velocity;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safe_commands");
    SafeCommands safe_commands;

    ros::Rate rate(50); // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
