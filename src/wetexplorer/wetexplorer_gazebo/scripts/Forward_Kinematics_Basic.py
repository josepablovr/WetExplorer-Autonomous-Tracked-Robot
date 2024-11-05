#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from math import cos, sin

class ForwardKinematics:
    def __init__(self):
        rospy.init_node('forward_kinematics_node')

        # Parameters
        self.radius_mid = 0.083  # Radius of the mid-front wheels
        self.b = 0.5 # Distance between tracks

        # Subscriber to /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Publisher for the calculated odometry
        self.odom_pub = rospy.Publisher('/odometry/forward_kinematics', Odometry, queue_size=10)

        # Internal state for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.frequency = 15
        # Timestamp
        self.last_time = rospy.Time.now()

        # Set the frequency limit to 15 Hz
        self.rate = rospy.Rate(self.frequency )

    def joint_states_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Limit the callback to 15 Hz
        if dt < 1/self.frequency:
            return

        # Extract velocities for the mid-front wheels
        try:
            mid_front_left_idx = msg.name.index('power_pulley_left_joint')
            mid_front_right_idx = msg.name.index('power_pulley_right_joint')

            omega_L = msg.velocity[mid_front_left_idx]
            omega_R = msg.velocity[mid_front_right_idx]

            # Compute forward kinematics
            V_x, theta_dot = self.forward_kinematics(omega_L, omega_R)

            # Update position and orientation based on the velocities
            self.update_odometry(V_x, theta_dot)

        except ValueError as e:
            rospy.logwarn(f"Joint not found: {e}")

        # Sleep to maintain 15 Hz rate
        self.rate.sleep()

    def forward_kinematics(self, omega_L, omega_R):
        # Calculate linear and angular velocities
        V_x = self.radius_mid / 2 * (omega_L + omega_R)
        theta_dot = self.radius_mid / self.b * (omega_R - omega_L)
        return V_x, theta_dot

    def update_odometry(self, V_x, theta_dot):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Update the robot's position and orientation
        delta_x = V_x * dt * cos(self.theta)
        delta_y = V_x * dt * sin(self.theta)
        delta_theta = theta_dot * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Prepare the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quat)

        # Set the velocity
        odom_msg.twist.twist.linear.x = V_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = theta_dot

        # Set pose covariance (example values, adjust as necessary)
        odom_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 99999, 0, 0, 0,
                                    0, 0, 0, 99999, 0, 0,
                                    0, 0, 0, 0, 99999, 0,
                                    0, 0, 0, 0, 0, 0.05]

        # Set twist covariance (example values, adjust as necessary)
        odom_msg.twist.covariance = [0.001, 0, 0, 0, 0, 0,
                                     0, 0.1, 0, 0, 0, 0,
                                     0, 0, 99999, 0, 0, 0,
                                     0, 0, 0, 99999, 0, 0,
                                     0, 0, 0, 0, 99999, 0,
                                     0, 0, 0, 0, 0, 0.05]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Update the last timestamp
        self.last_time = current_time

        rospy.loginfo(f"Published odometry: V_x: {V_x:.4f}, theta_dot: {theta_dot:.4f}")

if __name__ == '__main__':
    try:
        forward_kinematics = ForwardKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass