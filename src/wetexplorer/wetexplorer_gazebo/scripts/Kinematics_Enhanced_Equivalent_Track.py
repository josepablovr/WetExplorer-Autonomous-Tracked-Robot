#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState,Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from scipy.interpolate import interp1d

class EnhancedKinematicsNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('enhanced_kinematics_node')

        # Parameters directly defined
        self.r = 0.075  # radius of sprockets (meters)
        self.b = 0.6108  # distance between tracks (meters)
        self.alpha = 0.0  # slip angle
        self.i_L = 0.0  # slip on the left track
        self.i_R = 0.0  # slip on the right track
        self.theta = 0.0  # Heading from odometry
        self.beta = 0.0   # Real heading angle
        self.track_separation_correction = 1

        self.b = self.b*self.track_separation_correction
        self.angular_speed_reading = 0

        self.alpha = 0.1

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.real_heading_callback)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.delta_pub = rospy.Publisher('/delta_v_and_corrected_separation', Vector3, queue_size=10)


        # Publisher
        self.odom_pub = rospy.Publisher('/odometry/enhanced_kinematics', Odometry, queue_size=10)

        rospy.loginfo("Enhanced Kinematics Node Initialized")

    def imu_callback(self, msg):             
        self.angular_speed_reading = self.alpha * msg.angular_velocity.z + (1 - self.alpha) * self.angular_speed_reading
    
    def interpolate_corrected_separation(self,delta_V):
        # Define the known data points
        delta_V_points =              np.array([0.0,   0.1, 0.2, 0.3,  0.4, 0.5,   0.6, 0.7, 0.85])
        corrected_separation_points = np.array([0.61, 1.58, 1.5, 1.45, 1.1, 0.96, 0.85, 0.8, 0.8])

        # Create an interpolation function
        interpolation_function = interp1d(delta_V_points, corrected_separation_points, kind='linear', fill_value="extrapolate")

        # Return the interpolated corrected separation for the given delta_V
        return float(interpolation_function(delta_V))

    def joint_state_callback(self, msg):
        # Extract rotational velocities of the tracks
        omega_L = msg.velocity[2]  # Left track rotational velocity
        omega_R = msg.velocity[1]  # Right track rotational velocity

        # Calculate velocities
        V_x = self.r / (2 * np.cos(self.alpha)) * (omega_L * (1 - self.i_L) + omega_R * (1 - self.i_R))
        theta_dot = self.r / self.b * (omega_L * (1 - self.i_L) - omega_R * (1 - self.i_R))


        #Compute Differential Velocities
        V_L = self.r  * omega_L 
        V_R = self.r  * omega_R

        delta_V = abs(V_L-V_R)

        angular_ratio = theta_dot/self.angular_speed_reading

        

        corrected_separation = self.r / self.angular_speed_reading * (omega_L * (1 - self.i_L) - omega_R * (1 - self.i_R))
        

        if delta_V <= 0.1:
            corrected_separation = 0.6108
        rospy.loginfo("delta_V: %.2f, corrected_separation: %.2f", delta_V, corrected_separation)
        delta_msg = Vector3()
        delta_msg.x = round(delta_V, 2)
        delta_msg.y = round(corrected_separation, 2)
        delta_msg.z = 0.0  # If you need a third value, otherwise keep it as 0.0
        self.delta_pub.publish(delta_msg)


        corrected_separation = self.interpolate_corrected_separation(delta_V)
        theta_dot = self.r / corrected_separation * (omega_L * (1 - self.i_L) - omega_R * (1 - self.i_R))

        

        

        # Compute Instantaneous Center of Rotation (X_ICR) using V_y
        if theta_dot != 0:
            V_y = 0.0  # Assuming no lateral slip for simplicity
            X_ICR = -V_y / theta_dot
        else:
            X_ICR = 0.0

        # Compute the rotation matrix S_q
        S_q = np.array([[np.cos(self.theta), X_ICR * np.sin(self.theta)],
                        [np.sin(self.theta), -X_ICR * np.cos(self.theta)],
                        [0, 1]])

        # Compute the final velocities
        V_t = np.array([V_x, theta_dot])
        q_dot = np.dot(S_q, V_t)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "base_link"

        # Set the position (x, y)
        odom_msg.pose.pose.position.x = 0.0  # No position update in this example
        odom_msg.pose.pose.position.y = 0.0

        # Set the orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation.z = np.sin(self.beta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(self.beta / 2.0)

        # Set the velocity
        odom_msg.twist.twist.linear.x = V_x
        odom_msg.twist.twist.linear.y = V_y  # No lateral velocity in this example
        odom_msg.twist.twist.angular.z = theta_dot

        # Publish the message
        self.odom_pub.publish(odom_msg)

    def odometry_callback(self, msg):
        # Update the theta (heading from odometry)
        orientation_q = msg.pose.pose.orientation
        self.theta = 2 * np.arctan2(orientation_q.z, orientation_q.w)

    def real_heading_callback(self, msg):
        # Update the beta (real heading)
        orientation_q = msg.pose.pose.orientation
        self.beta = 2 * np.arctan2(orientation_q.z, orientation_q.w)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EnhancedKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
