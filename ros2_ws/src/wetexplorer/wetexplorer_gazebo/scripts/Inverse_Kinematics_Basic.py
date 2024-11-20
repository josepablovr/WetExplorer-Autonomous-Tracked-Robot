#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class WheelController:
    def __init__(self):
        rospy.init_node('wheel_controller_node')

        # Parameters
        self.b = 0.6108  # distance between tracks (meters)
        self.radius_front = 0.075
        self.radius_mid = 0.02
        self.radius_rear = 0.09

        self.radius_sprocket = 0.083

        # Limits for linear and angular velocities and accelerations
        self.max_linear_velocity = 5.0       # m/s
        self.max_linear_acceleration = 1.0   # m/s^2
        self.max_angular_velocity = 2.0      # rad/s
        self.max_angular_acceleration = 2.0  # rad/s^2

        # Initialize previous velocity and time for acceleration calculation
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.prev_time = rospy.Time.now()

        # Publishers for each wheel's velocity controller
        self.front_left_pub = rospy.Publisher('/front_left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.front_right_pub = rospy.Publisher('/front_right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.rear_left_pub = rospy.Publisher('/rear_left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.rear_right_pub = rospy.Publisher('/rear_right_wheel_velocity_controller/command', Float64, queue_size=10)

        self.mid_left_pub = rospy.Publisher('/command/pseudowheels_left', Float64, queue_size=10)
        self.mid_right_pub = rospy.Publisher('/command/pseudowheels_right', Float64, queue_size=10)


        self.sprocket_left_pub = rospy.Publisher('/left_sprocket_controller/command', Float64, queue_size=10)
        self.sprocket_right_pub = rospy.Publisher('/right_sprocket_controller/command', Float64, queue_size=10)



        self.vel_cmd_out = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the velocity command
        rospy.Subscriber('/robo/cmd_vel', Twist, self.cmd_vel_callback)

    def inverse_kinematics(self, V_x, theta_dot, r):
        # Inverse kinematics to compute wheel angular velocities
        omega_L = (V_x / r) - (self.b * theta_dot / (2 * r))
        omega_R = (V_x / r) + (self.b * theta_dot / (2 * r))
        return omega_L, omega_R

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

    def cmd_vel_callback(self, msg):
        # Get the current time
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        # Get linear and angular velocities from the cmd_vel message
        V_x = msg.linear.x
        theta_dot = msg.angular.z

        # Clamp the incoming velocities to their respective limits
        #V_x = self.clamp_velocity(V_x, self.max_linear_velocity)
        #theta_dot = self.clamp_velocity(theta_dot, self.max_angular_velocity)

        # Apply rate limiting to respect acceleration limits
        #V_x = self.rate_limit_velocity(V_x, self.prev_linear_velocity, self.max_linear_acceleration, dt)
        #theta_dot = self.rate_limit_velocity(theta_dot, self.prev_angular_velocity, self.max_angular_acceleration, dt)

        # Publish the control actions
        cmd_vel = Twist()
        cmd_vel.linear.x = V_x
        cmd_vel.angular.z = theta_dot

        self.vel_cmd_out.publish(cmd_vel)


        # Compute angular velocities for each set of wheels
        front_left_vel, front_right_vel = self.inverse_kinematics(V_x, theta_dot, self.radius_front)
        mid_left_vel, mid_right_vel = self.inverse_kinematics(V_x, theta_dot, self.radius_mid)        
        rear_left_vel, rear_right_vel = self.inverse_kinematics(V_x, theta_dot, self.radius_rear)
        sprocket_left_vel, sprocket_right_vel = self.inverse_kinematics(V_x, theta_dot, self.radius_sprocket)

        rospy.loginfo("Front Left: %.2f, Front Right: %.2f", front_left_vel, front_right_vel)

        # Publish the computed angular velocities to the corresponding topics
        self.front_left_pub.publish(front_left_vel)
        self.front_right_pub.publish(front_right_vel)
        self.mid_left_pub.publish(mid_left_vel)
        self.mid_right_pub.publish(mid_right_vel)    
        self.rear_left_pub.publish(rear_left_vel)
        self.rear_right_pub.publish(rear_right_vel)
        self.sprocket_left_pub.publish(sprocket_left_vel)
        self.sprocket_right_pub.publish(sprocket_right_vel)

        # Store the current velocities and time for the next callback
        self.prev_linear_velocity = V_x
        self.prev_angular_velocity = theta_dot
        self.prev_time = current_time

if __name__ == '__main__':
    try:
        wheel_controller = WheelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
