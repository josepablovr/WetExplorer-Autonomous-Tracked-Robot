#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math

class SlipCalculationNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('slip_calculation', anonymous=True)

        # Parameters
        self.radius_sprocket = 0.083 # Given radius of the sprocket
        self.track_width = 0.5       # Modify based on your robot's actual track width (b)
        self.alpha = 0.45             # Low-pass filter constant (0 < alpha < 1)
        self.track_separation = 0.6108
        # Initialize variables to store filtered data
        self.power_pulley_left_joint = 0.0
        self.power_pulley_right_joint = 0.0
        self.pasive_wheel_right = 0.0
        self.pasive_wheel_left = 0.0
        self.linear_velocity_filtered = 0.0
        self.angular_velocity_filtered = 0.0
        self.x_speed_filtered = 0.0
        self.y_speed_filtered = 0.0

        self.radius_passive_wheel = 0.075
    
        # Publisher
        self.slip_pub = rospy.Publisher('/slippage', Float32MultiArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        self.odom_pub = rospy.Publisher('/odometry/slippage', Odometry, queue_size=10)

    def joint_state_callback(self, msg):
        # Get the velocities of the power pulley joints
        try:
            self.power_pulley_left_joint = msg.velocity[msg.name.index("power_pulley_left_joint")]
            self.power_pulley_right_joint = msg.velocity[msg.name.index("power_pulley_right_joint")]

            self.pasive_wheel_right = msg.velocity[msg.name.index("front_right_wheel_joint")]
            self.pasive_wheel_left = msg.velocity[msg.name.index("front_left_wheel_joint")]
        except ValueError:
            rospy.logwarn("Could not find joint names in /joint_states message.")

    def odometry_callback(self, msg):
        # Apply low-pass filter to the linear and angular velocities
        self.linear_velocity_filtered = self.alpha * msg.twist.twist.linear.x + (1 - self.alpha) * self.linear_velocity_filtered
        self.angular_velocity_filtered = self.alpha * msg.twist.twist.angular.z + (1 - self.alpha) * self.angular_velocity_filtered
        self.x_speed_filtered = self.alpha * msg.twist.twist.linear.x + (1 - self.alpha) * self.x_speed_filtered
        self.y_speed_filtered = self.alpha * msg.twist.twist.linear.y + (1 - self.alpha) * self.y_speed_filtered

        self.calculate_slippage()

    def calculate_slippage(self):
        current_time = rospy.Time.now()
        # Calculate right and left sprocket velocities
        right_sprocket_velocity = self.pasive_wheel_right
        left_sprocket_velocity = self.pasive_wheel_left

        self.radius_sprocket = self.radius_passive_wheel

        # Calculate right slippage
        if abs(right_sprocket_velocity*self.radius_sprocket) > 0.01:
            percentage = abs((self.linear_velocity_filtered + (self.track_width / 2) * self.angular_velocity_filtered) / (self.radius_sprocket * right_sprocket_velocity))
            slippage_right = 1 - max(0.0, min(1.0, percentage))
        else:
            slippage_right = 0

        # Calculate left slippage
        if abs(left_sprocket_velocity*self.radius_sprocket) > 0.01:
            percentage =  abs((self.linear_velocity_filtered - (self.track_width / 2) * self.angular_velocity_filtered) / (self.radius_sprocket * left_sprocket_velocity))
            slippage_left = 1 - max(0.0, min(1.0, percentage))
        else:
            slippage_left = 0




        # Clamp slippage values to [0, 1]
        
        slippage_left = max(-1.0, min(1.0, slippage_left))




        # Calculate alpha
        if self.x_speed_filtered > 0.01:
            alpha = math.atan2(self.y_speed_filtered, self.x_speed_filtered)
        else:
            alpha = 0.0


        self.track_separation = 0.5
        V = self.radius_sprocket/(2*math.cos(alpha))*(left_sprocket_velocity*(1-slippage_left) + right_sprocket_velocity*(1-slippage_right))
        theta = self.radius_sprocket/self.track_separation *(-left_sprocket_velocity*(1-slippage_left) + right_sprocket_velocity*(1-slippage_right))
        
        b = theta/self.angular_velocity_filtered
        # Prepare and publish the message
        slip_msg = Float32MultiArray()
        slip_msg.data = [slippage_right, slippage_left, alpha, b, self.pasive_wheel_right*self.radius_passive_wheel, right_sprocket_velocity*self.radius_sprocket]
        self.slip_pub.publish(slip_msg)



        # Prepare the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = theta
        self.odom_pub.publish(odom_msg)




if __name__ == '__main__':
    try:
        slip_node = SlipCalculationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
