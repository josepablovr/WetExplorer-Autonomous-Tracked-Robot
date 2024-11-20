#!/usr/bin/env python3

import rospy
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_srvs.srv import SetBool, SetBoolResponse
import tf.transformations
from tf.transformations import euler_from_quaternion

class KinematicController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('kinematic_controller', anonymous=True)
        
        # Parameters for the controller
        self.goal_x = 1.0
        self.goal_y = 1.0
        self.K_alpha = 4.0
        self.K_beta = -0.7
        self.Kv = 1.5
        self.tolerance = 0.25 # 20 cm
        
        # Maximum speeds
        self.max_linear_speed = 0.33  # meters per second
        self.max_angular_speed = 0.33  # radians per second

        # Initialize the current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publisher for the chamber_link position in the odom frame
        self.pose_pub = rospy.Publisher('/odometry/tcp', PoseStamped, queue_size=10)
        
        # Subscriber for odometry data
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odometry_callback)

        # Service to check if the goal has been reached
        rospy.Service('/goal_reached', SetBool, self.goal_reached_service)

        # TF2 Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    
    def odometry_callback(self, msg):
        try:
            # Get the transformation from base_link to odom
            transform_base_to_odom = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))

            # Get the transformation from base_link to chamber_link
            transform_base_to_chamber = self.tf_buffer.lookup_transform('chamber_link', 'base_link', rospy.Time(0))

            # Extract the base_link position and orientation from the odometry message
            base_link_pose = msg.pose.pose
            x_base = base_link_pose.position.x
            y_base = base_link_pose.position.y
            orientation_q_base = base_link_pose.orientation

            # Convert quaternion to yaw (theta)
            _, _, theta_base = euler_from_quaternion([orientation_q_base.x, orientation_q_base.y, orientation_q_base.z, orientation_q_base.w])
 

            # Compute the rotation matrix for base_link to odom
            R_base_to_odom = np.array([
                [np.cos(theta_base), -np.sin(theta_base)],
                [np.sin(theta_base), np.cos(theta_base)]
            ])
   

            # Extract the translation and rotation components from base_link to chamber_link
            x_chamber = transform_base_to_chamber.transform.translation.x
            y_chamber = transform_base_to_chamber.transform.translation.y
            orientation_q_chamber = transform_base_to_chamber.transform.rotation
            _, _, theta_chamber = euler_from_quaternion([orientation_q_chamber.x, orientation_q_chamber.y, orientation_q_chamber.z, orientation_q_chamber.w])

            # Compute the position of chamber_link in the odom frame
            x_chamber_in_odom = x_base + -R_base_to_odom[0, 0] * x_chamber - R_base_to_odom[0, 1] * y_chamber
            y_chamber_in_odom = y_base + -R_base_to_odom[1, 0] * x_chamber - R_base_to_odom[1, 1] * y_chamber

            # Combine the orientation
            theta_chamber_in_odom = theta_base
           
            # Publish the position of the chamber_link in the odom frame
            chamber_pose_in_odom = PoseStamped()
            chamber_pose_in_odom.header.stamp = rospy.Time.now()
            chamber_pose_in_odom.header.frame_id = 'odom'
            chamber_pose_in_odom.pose.position.x = x_chamber_in_odom
            chamber_pose_in_odom.pose.position.y = y_chamber_in_odom
            chamber_pose_in_odom.pose.position.z = 0  # Assuming 2D motion
            quaternion_chamber = tf.transformations.quaternion_from_euler(0, 0, theta_chamber_in_odom)
            chamber_pose_in_odom.pose.orientation.x = quaternion_chamber[0]
            chamber_pose_in_odom.pose.orientation.y = quaternion_chamber[1]
            chamber_pose_in_odom.pose.orientation.z = quaternion_chamber[2]
            chamber_pose_in_odom.pose.orientation.w = quaternion_chamber[3]

            # Publish the pose
            self.pose_pub.publish(chamber_pose_in_odom)

            # Compute the control commands
            linear_speed_chamber, angular_speed_chamber = self.compute_control_commands(
                x_chamber_in_odom, y_chamber_in_odom, theta_base,
                self.goal_x, self.goal_y, self.K_alpha, self.K_beta, self.Kv
            )

            # Create a twist message for the chamber_link frame
            cmd_vel_chamber = Twist()
            cmd_vel_chamber.linear.x = linear_speed_chamber
            cmd_vel_chamber.angular.z = angular_speed_chamber

            # Transform the control commands back to the base_link frame
            cmd_vel_base = self.transform_cmd_vel_to_base(cmd_vel_chamber, transform_base_to_odom)

            # Publish the control commands if the goal has not been reached
            if not self.goal_reached:
                self.cmd_vel_pub.publish(cmd_vel_base)

            # Check if the goal has been reached
            self.goal_reached = self.is_goal_reached(x_chamber_in_odom, y_chamber_in_odom, self.goal_x, self.goal_y, self.tolerance)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform odometry or control commands between frames: {e}")



    def transform_cmd_vel_to_base(self, cmd_vel_chamber, transform):
        """
        Transform the cmd_vel from the chamber_link frame back to the base_link frame.
        """
        # Extract the rotation (quaternion) from the transform
        rotation_q = transform.transform.rotation
        rotation_matrix = tf.transformations.quaternion_matrix([rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w])

        # Transform linear velocity
        linear_velocity_chamber = [cmd_vel_chamber.linear.x, 0, 0]  # 2D motion assumption
        linear_velocity_base = rotation_matrix[:3, :3].dot(linear_velocity_chamber)

        # Transform angular velocity (only z component is considered since it's 2D)
        angular_velocity_chamber = [0, 0, cmd_vel_chamber.angular.z]
        angular_velocity_base = rotation_matrix[:3, :3].dot(angular_velocity_chamber)

        # Create a new Twist message for base_link frame
        cmd_vel_base = Twist()
        cmd_vel_base.linear.x = linear_velocity_base[0]
        cmd_vel_base.linear.y = linear_velocity_base[1]
        cmd_vel_base.angular.z = angular_velocity_base[2]

        return cmd_vel_base

    def compute_control_commands(self, x_actual, y_actual, yaw_actual, x_goal, y_goal, K_alpha, K_beta, Kv):
        """
        Compute the control inputs (linear speed and angular speed) for a mobile robot.
        """
        # Compute the difference in position
        dx = x_goal - x_actual
        dy = y_goal - y_actual
        
        # Compute the desired orientation (angle to the goal) - beta
        beta = math.atan2(dy, dx)
        
        # Compute alpha: the difference between beta and the current yaw (theta)
        alpha = beta - yaw_actual
        
        # Normalize alpha to the range [-pi, pi]
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        
        # Compute the magnitude of the error (distance to the goal)
        error_magnitude = math.sqrt(dx**2 + dy**2)
        
        # Compute the control outputs
        angular_speed = K_alpha * alpha + K_beta * beta
        linear_speed = Kv * error_magnitude
        
        # Apply saturation limits to the control outputs
        linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
        angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
        
        return linear_speed, angular_speed

    def is_goal_reached(self, x_actual, y_actual, x_goal, y_goal, tolerance):
        """
        Check if the goal position has been reached within the specified tolerance.
        """
        error_magnitude = math.sqrt((x_goal - x_actual)**2 + (y_goal - y_actual)**2)
        return error_magnitude < tolerance

    def goal_reached_service(self, req):
        """
        Service callback to check if the goal has been reached.
        """
        response = SetBoolResponse()
        response.success = self.goal_reached
        response.message = "Goal reached!" if self.goal_reached else "Goal not reached yet."
        return response

    def run(self):
        # Spin to keep the node running
        rospy.spin()

if __name__ == '__main__':
    controller = KinematicController()
    controller.run()
