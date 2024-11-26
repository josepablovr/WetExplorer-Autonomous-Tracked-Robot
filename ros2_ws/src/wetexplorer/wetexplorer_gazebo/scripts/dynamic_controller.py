#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class VelocityController:
    def __init__(self):
        rospy.init_node('velocity_controller', anonymous=True)

        # Parameters
        self.Kpv = 0.2
        self.Kpw = 0.4
        self.Kiv = 0.0
        self.Kiw = 0.5
        self.FFv = 0.9 #0.9
        self.FFw = 0.5 #0.5
        self.alpha = 0.3


        self.Kpv = 0.0
        self.Kpw = 0.0
        self.Kiv = 0.0
        self.Kiw = 0.0
        self.FFv = 1.0#0.9
        self.FFw = 1.0 #0.5
        self.alpha = 0.3

        # Initialize errors and integrals
        self.linear_velocity_error = 0.0
        self.angular_velocity_error = 0.0
        self.linear_integral = 0.0
        self.angular_integral = 0.0

        # Initialize filtered velocities
        self.filtered_linear_velocity = 0.0
        self.filtered_angular_velocity = 0.0

        self.min_control_action = 0.025

        self.anti_wind_up = 0.3

        # Limits for linear and angular velocities and accelerations
        self.max_linear_velocity = 2.0      # m/s
        self.max_linear_acceleration = 0.5   # m/s^2
        self.max_angular_velocity = 1.4      # rad/s
        self.max_angular_acceleration = 0.5  # rad/s^2

        # Initialize previous velocity and time for acceleration calculation
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.prev_time = rospy.Time.now()


        self.vel_cmd_out = rospy.Publisher('/vel_cmd_limited', Twist, queue_size=10)

        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/WetExplorer/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel_out', Twist, self.commands_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        # Subscriber to the velocity command
  

        self.command = Twist()
        self.odom = Odometry()

    def commands_callback(self, msg):
        self.command = msg

    def odometry_callback(self, msg):
        self.odom = msg

    def clamp_velocity(self, velocity, max_velocity):
        # Clamps the velocity to the maximum allowable limit
        return max(min(velocity, max_velocity), -max_velocity)

    def rate_limit_velocity(self, current_velocity, previous_velocity, max_acceleration, dt):
        # Limits the rate of change of velocity to satisfy acceleration limits
        max_delta_velocity = max_acceleration * dt
        delta_velocity = current_velocity - previous_velocity
        if abs(delta_velocity) > max_delta_velocity:
            current_velocity = previous_velocity + math.copysign(max_delta_velocity, delta_velocity)
        return current_velocity

    def control_loop(self):
        rate = rospy.Rate(50)  # 10 Hz

        while not rospy.is_shutdown():
            # Get the current command velocities


            # Get the current time
            current_time = rospy.Time.now()
            dt = (current_time - self.prev_time).to_sec()

            # Get linear and angular velocities from the cmd_vel message
            V_x = self.command.linear.x
            theta_dot = self.command.angular.z

            # Clamp the incoming velocities to their respective limits
            V_x = self.clamp_velocity(V_x, self.max_linear_velocity)
            theta_dot = self.clamp_velocity(theta_dot, self.max_angular_velocity)

            # Apply rate limiting to respect acceleration limits
            V_x = self.rate_limit_velocity(V_x, self.prev_linear_velocity, self.max_linear_acceleration, dt)
            theta_dot = self.rate_limit_velocity(theta_dot, self.prev_angular_velocity, self.max_angular_acceleration, dt)

            # Publish the control actions
            cmd_vel = Twist()
            cmd_vel.linear.x = V_x
            cmd_vel.angular.z = theta_dot

            self.vel_cmd_out.publish(cmd_vel)


            linear_velocity_command = V_x
            angular_velocity_command = theta_dot

            # Get the current odometry velocities
            linear_velocity_reading = self.odom.twist.twist.linear.x
            angular_velocity_reading = self.odom.twist.twist.angular.z

            # Apply low-pass filter
            self.filtered_linear_velocity = (
                self.alpha * linear_velocity_reading + (1 - self.alpha) * self.filtered_linear_velocity
            )
            self.filtered_angular_velocity = (
                self.alpha * angular_velocity_reading + (1 - self.alpha) * self.filtered_angular_velocity
            )

            # Calculate errors
            self.linear_velocity_error = linear_velocity_command - self.filtered_linear_velocity
            self.angular_velocity_error = angular_velocity_command - self.filtered_angular_velocity

            rospy.loginfo(f"Linear Error V_x: {self.linear_velocity_error :.4f}, Angular Error: {self.angular_velocity_error:.4f}")
            
            # Integrate errors (Integral term)
            self.linear_integral += self.linear_velocity_error
            self.angular_integral += self.angular_velocity_error

            # Saturate integrals to prevent windup
            self.linear_integral = max(min(self.linear_integral, self.anti_wind_up), -self.anti_wind_up)
            self.angular_integral = max(min(self.angular_integral, self.anti_wind_up), -self.anti_wind_up)

            # Calculate control actions
            linear_velocity_control_action = (
                self.Kpv * self.linear_velocity_error +
                self.Kiv * self.linear_integral +
                self.FFv * linear_velocity_command
            )


            if abs(linear_velocity_control_action)<= self.min_control_action:
                linear_velocity_control_action = 0.0


            angular_velocity_control_action = (
                self.Kpw * self.angular_velocity_error +
                self.Kiw * self.angular_integral +
                self.FFw * angular_velocity_command
            )
            if abs(angular_velocity_control_action)<= self.min_control_action:
                angular_velocity_control_action = 0.0

            # Publish the control actions
            cmd_vel = Twist()
    

            cmd_vel.linear.x = linear_velocity_control_action
            cmd_vel.angular.z = angular_velocity_control_action

            self.cmd_vel_pub.publish(cmd_vel)
            #self.cmd_vel_pub2.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = VelocityController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
