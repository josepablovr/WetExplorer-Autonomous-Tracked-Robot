#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityCompensator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velocity_compensator')

        # Compensator gains
        self.Kv = 1.0  # Gain for linear velocity compensation
        self.Komega = 2.0  # Gain for angular velocity compensation

        # Low-pass filter parameter
        self.alpha = 0.1  # Smoothing factor

        # Previous filtered velocities (initialize to zero)
        self.filtered_vel_linear_x = 0.0
        self.filtered_vel_angular_z = 0.0

        # Commanded velocities (from /commands topic)
        self.cmd_vel = Twist()

        # Publisher for the corrected velocities
        self.corrected_cmd_vel_pub = rospy.Publisher('/joy_teleop/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.commands_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)

    def commands_callback(self, msg):
        self.cmd_vel = msg

    def odometry_callback(self, msg):
        # Extract the actual linear and angular velocities
        actual_vel_linear_x = msg.twist.twist.linear.x
        actual_vel_angular_z = msg.twist.twist.angular.z

        # Apply low-pass filter to linear and angular velocities
        self.filtered_vel_linear_x = self.alpha * actual_vel_linear_x + (1 - self.alpha) * self.filtered_vel_linear_x
        self.filtered_vel_angular_z = self.alpha * actual_vel_angular_z + (1 - self.alpha) * self.filtered_vel_angular_z

        # Compute the error between commanded and filtered velocities
        error_v = self.cmd_vel.linear.x - self.filtered_vel_linear_x
        error_omega = self.cmd_vel.angular.z - self.filtered_vel_angular_z

        # Apply compensation
        corrected_cmd_vel = Twist()
        corrected_cmd_vel.linear.x = self.cmd_vel.linear.x + self.Kv * error_v
        corrected_cmd_vel.angular.z = self.cmd_vel.angular.z + self.Komega * error_omega

        # Publish the corrected velocities
        self.corrected_cmd_vel_pub.publish(corrected_cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    compensator = VelocityCompensator()
    compensator.run()
