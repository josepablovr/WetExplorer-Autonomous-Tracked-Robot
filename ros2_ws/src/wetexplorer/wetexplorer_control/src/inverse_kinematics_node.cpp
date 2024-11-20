#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>  // For publishing an array of angular velocities
#include <cmath>  // for M_PI
#include <sstream>
#include <vector>
class RobotControl
{
public:
    RobotControl()
    {
        // Initialize the ROS node and set up subscriber and publisher
        cmd_vel_sub = n.subscribe("/WetExplorer/cmd_vel", 10, &RobotControl::cmdVelCallback, this);
        motor_cmd_pub = n.advertise<geometry_msgs::Twist>("/robo/cmd_vel", 10);
         // Publisher for angular velocities (omega_L, omega_R)
        angular_velocity_pub = n.advertise<std_msgs::Float32MultiArray>("/angular_velocity_cmd", 10);
    }

    // Callback function to compute inverse kinematics
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& vel)
    {
        double V_x = vel->linear.x;  // Linear velocity
        double theta_dot = vel->angular.z;  // Angular velocity
        
        // Compute wheel velocities using inverse kinematics
        double omega_L, omega_R;
        inverseKinematics(V_x, theta_dot, radius_sprocket, omega_L, omega_R);

         // Convert angular velocities (rad/s) to RPM
        double omega_L_rpm = (omega_L * 60) / (2 * M_PI);
        double omega_R_rpm = (omega_R * 60) / (2 * M_PI);

        // Multiply by gear ratio
        omega_L_rpm *= gear_ratio;
        omega_R_rpm *= gear_ratio;

        // Prepare and publish angular velocities (omega_L and omega_R) as a Float32MultiArray
        std_msgs::Float32MultiArray angular_velocity_cmd;
        angular_velocity_cmd.data.push_back(omega_R_rpm);
        angular_velocity_cmd.data.push_back(omega_L_rpm);
        angular_velocity_pub.publish(angular_velocity_cmd);  // Publish the array



              // Map to percentage of vel_max_rpm
        double omega_L_percentage = (omega_L_rpm / vel_max_rpm) * 1000.0;
        double omega_R_percentage = (omega_R_rpm / vel_max_rpm) * 1000.0;



        // Clamp values to the range [-100, 100] 
        omega_L_percentage = std::min(1000.0, std::max(-1000.0, omega_L_percentage));
        omega_R_percentage = std::min(1000.0, std::max(-1000.0, omega_R_percentage));


        // Convert to integers (round if necessary)
        int omega_L_int = static_cast<int>(std::round(omega_L_percentage));
        int omega_R_int = static_cast<int>(std::round(omega_R_percentage));
        // Prepare the motor command message
        geometry_msgs::Twist motor_cmd;


        


        motor_cmd.linear.x = omega_R_int;  // Right motor command in percentage
        motor_cmd.angular.z = omega_L_int;  // Left motor command in percentage

      
        // Publish the motor commands
        motor_cmd_pub.publish(motor_cmd);

        // Print for debugging
        ROS_INFO("Published motor commands: R_motor = %.2f, L_motor = %.2f", omega_R, omega_L);
    }

    // Inverse kinematics function
    void inverseKinematics(double V_x, double theta_dot, double r, double& omega_L, double& omega_R)
    {
        // Inverse kinematics calculation
        omega_L = (V_x / r) - (b * theta_dot / (2 * r));
        omega_R = (V_x / r) + (b * theta_dot / (2 * r));
    }

private:
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub;  // Subscriber to /cmd_vel_out
    ros::Publisher motor_cmd_pub;  // Publisher to /robo/cmd_vel
     ros::Publisher angular_velocity_pub;
    // Constants
    const double radius_sprocket = 0.083;  // Radius of sprocket in meters
    const double track_separation = 0.6103;  // Radius of sprocket in meters
    const double b = 0.6108;  // Distance between tracks in meters
    // const double scaling_factor = 1.0;  // Example scaling factor (optional)

    double gear_ratio = 30;        // Gear ratio
    double vel_max_rpm = 1500;       // Maximum motor RPM
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");
    RobotControl robot_control;

    ros::spin();

    return 0;
}
