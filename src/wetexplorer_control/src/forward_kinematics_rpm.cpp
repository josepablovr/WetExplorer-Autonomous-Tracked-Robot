#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <vector>
#include <cmath>  // for M_PI
#include <tf/tf.h>

class ForwardKinematics {
public:
    ForwardKinematics() {
        // Initialize track separation (distance between left and right sprockets)
        tracks_separation = 0.6108;  // meters
        gear_ratio = 30;
        radius_sprocket = 0.0754;  // meters

        x = 0.0;
        y = 0.0;
        theta = 0.0;
        last_time = ros::Time::now();

        // Subscribe to the /robo/velocity topic
        velocity_sub = nh.subscribe("/robo/sys", 10, &ForwardKinematics::velocityCallback, this);

        // Publisher to the new /odometry/robo topic
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/forward_kinematics", 10);
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
        if (tokens.size() < 2) {
            ROS_WARN("Velocity message does not contain enough data.");
            return;
        }
        double omega_L_rpm = 0.0;
        double omega_R_rpm = 0.0;
        
        // Extract left and right angular velocities in RPM
        try{
          omega_L_rpm = std::stod(tokens[4]);
          omega_R_rpm = std::stod(tokens[3]);}
        catch(const std::invalid_argument& e){
          //ROS_ERROR("Invalid argument: %s" e.what());
          }

        // Convert RPM to rad/s
        double omega_L = (omega_L_rpm * 2 * M_PI) / 60;
        double omega_R = (omega_R_rpm * 2 * M_PI) / 60;

        // Convert angular velocities to linear velocities (m/s)
        double v_L = omega_L * radius_sprocket / gear_ratio;
        double v_R = omega_R * radius_sprocket / gear_ratio;

        // Compute linear velocity (V_x) and angular velocity (theta_dot)
        double V_x = (v_L + v_R) / 2;
        double theta_dot = (v_R - v_L) / tracks_separation;

        // Publish the odometry message
        publishOdometry(V_x, theta_dot);
    }

    void publishOdometry(double linear_velocity, double angular_velocity) {
        nav_msgs::Odometry odom_msg;
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();  // Time step

        // Update the robot's orientation
        theta += angular_velocity * dt;

        // Update the robot's position
        x += linear_velocity * dt * cos(theta);
        y += linear_velocity * dt * sin(theta);
 
        

        // Create the odometry message
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set the pose (position + orientation)
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        // Set the linear and angular velocities
        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        // Set covariance values
        for (int i = 0; i < 36; ++i) {
            if (i == 0) {
              
                if (abs(linear_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.000001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.0001; 
                }
            } else if (i == 7) {
                
                if (abs(linear_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.0001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.001; 
                }
                
            } else if (i == 35) {
                if (abs(angular_velocity) <= 0.05){
                  odom_msg.twist.covariance[i] = 0.00001; 
                }
                else {
                  odom_msg.twist.covariance[i] = 0.05;
                }
                
            } else if (i == 14 || i == 21 || i == 28) {
                odom_msg.twist.covariance[i] = 99999.0;
            } else {
                odom_msg.twist.covariance[i] = 0.0;
            }
        }

        // Publish the odometry message
        odom_pub.publish(odom_msg);

        // Update the last time
        last_time = current_time;
        }

private:
    ros::NodeHandle nh;
    ros::Subscriber velocity_sub;
    ros::Publisher odom_pub;

    double tracks_separation; // Distance between the tracks
    double gear_ratio;        // Gear ratio
    double radius_sprocket;   // Radius of the sprocket in meters

    double x, y, theta;
    ros::Time last_time;


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "forward_kinematics_node");
    ForwardKinematics fk;

    ros::spin();
    return 0;
}
