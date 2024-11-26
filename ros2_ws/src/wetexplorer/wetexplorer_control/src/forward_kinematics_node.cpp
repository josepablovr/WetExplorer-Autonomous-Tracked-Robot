#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <vector>

class ForwardKinematics {
public:
    ForwardKinematics() {
        // Initialize track separation (distance between left and right sprockets)
        tracks_separation = 0.6108;  // meters

        // Subscribe to the /robo/velocity topic
        velocity_sub = nh.subscribe("/robo/velocity", 10, &ForwardKinematics::velocityCallback, this);

        // Publisher to the new /odometry/robo topic
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/robo", 10);
    }

    void velocityCallback(const std_msgs::String::ConstPtr& msg) {
        // Parse the string data from the message
        std::string data = msg->data;
        std::vector<std::string> tokens;
        std::stringstream ss(data);
        std::string token;

        // Split the string by commas
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        // Ensure there are enough elements in the parsed string
        if (tokens.size() < 9) {
            ROS_WARN("Velocity message does not contain enough data.");
            return;
        }

        // Extract the last two values: linear and angular velocity
        double linear_velocity = std::stod(tokens[tokens.size() - 2]);  // Second-to-last value
        double angular_velocity = std::stod(tokens[tokens.size() - 1]); // Last value

        // Publish the odometry message
        publishOdometry(linear_velocity, angular_velocity);
    }

    void publishOdometry(double linear_velocity, double angular_velocity) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";  // Set child 0.1frame as base_link

        // Set the linear and angular velocities in the odometry message
        odom_msg.twist.twist.linear.x = -linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        for (int i = 0; i < 36; ++i) {
            if (i == 0) {  // Covariance for vx (index 0) 
                odom_msg.twist.covariance[i] = 0.0001;
            } 
            else if (i == 7) { //and vy (index 7)
                 odom_msg.twist.covariance[i] = 0.005; }
            else if (i == 35) {    // Covariance for wz (index 35)
                odom_msg.twist.covariance[i] = 0.1;
            } 
            
            else if (i == 14|i ==21|i==28) {    // Covariance for wz (index 35)
                odom_msg.twist.covariance[i] = 99999.0;
            }

            else {
                odom_msg.twist.covariance[i] = 0.0;
            }
        }

        // Publish the odometry message to /odometry/robo
        odom_pub.publish(odom_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber velocity_sub;
    ros::Publisher odom_pub;

    double tracks_separation; // Distance between the tracks
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "forward_kinematics_node");
    ForwardKinematics fk;

    ros::spin();
    return 0;
}
