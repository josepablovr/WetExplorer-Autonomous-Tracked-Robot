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
        
        # Initialize variables
        self.goal_x = None
        self.goal_y = None
        
        self.K_alpha = 0.15
        self.K_beta = -0.0
        self.Kv = 1.0
        self.tolerance = 0.05 # 20 cm
        
        # Maximum speeds
        self.max_linear_speed = 0.25  # meters per second
        self.max_angular_speed = 0.5 # radians per second

        # Initialize the current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False



        self.KPp = 1.0
        self.KPxte = 0.00 #5.00
        self.KPt = 0.1

        self.KIp = 0.00
        #self.KIxte = 0.50
        self.KIt = 0.0
        #self.KIp = 0.0
        self.KIxte = 0.0
        self.KIt = 0.0
        # Initialize integrals for position, orientation, and cross-track errors
        self.position_integral = 0.0
        self.orientation_integral = 0.0
        self.cross_track_integral = 0.0





        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/navigation/cmd_vel', Twist, queue_size=10)
        
        # Publisher for the chamber_link position in the odom frame
        self.pose_pub = rospy.Publisher('/odometry/tcp', PoseStamped, queue_size=10)
        
        # Subscriber for odometry data
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odometry_callback)

        # Service to check if the goal has been reached
        rospy.Service('/goal_reached', SetBool, self.goal_reached_service)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # TF2 Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def goal_callback(self, msg):
        # Extract goal coordinates
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        rospy.loginfo(f"Received new goal: x={self.goal_x}, y={self.goal_y}")

    def compute_transform(self, R1, T1, R2, T2):
        """
        Compute the combined transform between two frames.
        
        R1, T1: Rotation matrix and Translation vector from the origin frame to the intermediate frame.
        R2, T2: Rotation matrix and Translation vector from the intermediate frame to the target frame.
        
        Returns:
        R_combined: Combined rotation matrix.
        T_combined: Combined translation vector.
        """
        R_combined = np.dot(R1, R2)
        T_combined = np.dot(R1, T2) + T1
        return R_combined, T_combined
    
    def odometry_callback(self, msg):
        if self.goal_x is None or self.goal_y is None:
            return  # Skip if no goal is set
        
        try:

            # Extract the current pose of base_link in the odom frame from the odometry message
            base_link_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            base_link_orientation_q = msg.pose.pose.orientation
            # Get the transform between odom and base_link
            transform_odom_base_link = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))

            # Extract base_link to chamber_link transformation
            transform_base_link_chamber = self.tf_buffer.lookup_transform('base_link', 'prismatic_link', rospy.Time(0))

            # Decompose odom to base_link
            R_odom_base_link = tf.transformations.quaternion_matrix([transform_odom_base_link.transform.rotation.x,
                                                                    transform_odom_base_link.transform.rotation.y,
                                                                    transform_odom_base_link.transform.rotation.z,
                                                                    transform_odom_base_link.transform.rotation.w])[:3, :3]
            T_odom_base_link = np.array([transform_odom_base_link.transform.translation.x,
                                        transform_odom_base_link.transform.translation.y,
                                        transform_odom_base_link.transform.translation.z])

            # Decompose base_link to chamber_link
            R_base_link_chamber = tf.transformations.quaternion_matrix([transform_base_link_chamber.transform.rotation.x,
                                                                        transform_base_link_chamber.transform.rotation.y,
                                                                        transform_base_link_chamber.transform.rotation.z,
                                                                        transform_base_link_chamber.transform.rotation.w])[:3, :3]
            T_base_link_chamber = np.array([transform_base_link_chamber.transform.translation.x,
                                            transform_base_link_chamber.transform.translation.y,
                                            transform_base_link_chamber.transform.translation.z])

            # Compute the odom -> chamber_link transform
            R_odom_chamber, T_odom_chamber = self.compute_transform(R_odom_base_link, T_odom_base_link, 
                                                            R_base_link_chamber, T_base_link_chamber)
            

            # Compute odom -> chamber_link transform
            R_odom_chamber, T_odom_chamber = self.compute_transform(
                R_odom_base_link, base_link_position, R_base_link_chamber, T_base_link_chamber
            )



            # Compute the odom -> base_link transform based on the odom -> chamber_link
            R_chamber_base_link = R_base_link_chamber.T  # Inverse rotation
            T_chamber_base_link = -np.dot(R_chamber_base_link, T_base_link_chamber)  # Inverse translation
            
            R_odom_base_link_new, T_odom_base_link_new = self.compute_transform(R_odom_chamber, T_odom_chamber, 
                                                                        R_chamber_base_link, T_chamber_base_link)

            # Extract x, y, and orientation (theta) of chamber_link in the odom frame
            x_chamber_in_odom = T_odom_chamber[0]
            y_chamber_in_odom = T_odom_chamber[1]
            theta_base = np.arctan2(R_odom_base_link[1, 0], R_odom_base_link[0, 0])  # Orientation of base_link in odom frame

            """ linear_speed_chamber, angular_speed_chamber = self.compute_control_commands(
                x_chamber_in_odom, y_chamber_in_odom, theta_base,
                self.goal_x, self.goal_y, self.K_alpha, self.K_beta, self.Kv
            ) """
            
            linear_speed_chamber, angular_speed_chamber = self.compute_control_commands(x_chamber_in_odom, y_chamber_in_odom, theta_base, self.goal_x, self.goal_y, 0.0)
                
                # Create a twist message for the chamber_link frame
            cmd_vel_chamber = Twist()
            cmd_vel_chamber.linear.x = linear_speed_chamber
            cmd_vel_chamber.angular.z = angular_speed_chamber

            # Use this new transform to correct the cmd_vel before publishing
            #cmd_vel_base_corrected = self.transform_cmd_vel(cmd_vel_chamber, R_odom_base_link_new, T_odom_base_link_new)

            # Publish the corrected control commands


            # Publish the position of the chamber_link in the odom frame
            chamber_pose_in_odom = PoseStamped()
            chamber_pose_in_odom.header.stamp = rospy.Time.now()
            chamber_pose_in_odom.header.frame_id = 'odom'
            chamber_pose_in_odom.pose.position.x = x_chamber_in_odom
            chamber_pose_in_odom.pose.position.y = y_chamber_in_odom
            chamber_pose_in_odom.pose.position.z = 0  # Assuming 2D motion
            quaternion_chamber = tf.transformations.quaternion_from_euler(0, 0, theta_base)
            chamber_pose_in_odom.pose.orientation.x = quaternion_chamber[0]
            chamber_pose_in_odom.pose.orientation.y = quaternion_chamber[1]
            chamber_pose_in_odom.pose.orientation.z = quaternion_chamber[2]
            chamber_pose_in_odom.pose.orientation.w = quaternion_chamber[3]

            # Publish the pose
            self.pose_pub.publish(chamber_pose_in_odom)


            if not self.goal_reached:
                self.cmd_vel_pub.publish(cmd_vel_chamber)
            else:

                cmd_vel_chamber = Twist()
                cmd_vel_chamber.linear.x = 0.0
                cmd_vel_chamber.angular.z = 0.0
                rospy.loginfo(f"GOAL REACHED!")
                self.cmd_vel_pub.publish(cmd_vel_chamber)

            # Check if the goal has been reached
            self.goal_reached = self.is_goal_reached(x_chamber_in_odom, y_chamber_in_odom, self.goal_x, self.goal_y, self.tolerance)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform odometry or control commands between frames: {e}")



    def transform_cmd_vel(self, cmd_vel, R_transform, T_transform):
        v = np.array([cmd_vel.linear.x, cmd_vel.linear.y, 0])
        v_transformed = np.dot(R_transform, v[:2])
        
        cmd_vel_transformed = Twist()
        cmd_vel_transformed.linear.x = v_transformed[0]
        cmd_vel_transformed.linear.y = v_transformed[1]
        cmd_vel_transformed.angular.z = cmd_vel.angular.z
        
        return cmd_vel_transformed


    def compute_control_commands(self, x_actual, y_actual, yaw_actual, x_goal, y_goal, theta_desired):
        """
        Compute the control inputs (linear speed and angular speed) for a mobile robot using the specified control loop.
        """
        # Compute the difference in position
        dx = x_goal - x_actual
        dy = y_goal - y_actual
                   
        # Compute alpha: the difference between beta and the current yaw (theta)      
        alpha = math.atan2(dy, dx)
        beta = - yaw_actual + math.atan2(dy, dx)
        # Normalize alpha to the range [-pi, pi]
        ##alpha = (alpha + math.pi/2) % (2 * math.pi) - math.pi/2
        theta_desired = alpha
        # Normalize beta to the range [-pi, pi]
        beta = (beta + math.pi) % (2*math.pi) - math.pi
        
        # Compute the cross-track error
        cross_track_error = math.sqrt(dx**2 + dy**2) * math.sin(beta)

        # Compute the position error with the sign of the cross-track error
        position_error = math.sqrt(dx**2 + dy**2) 
        #3* math.copysign(1, cross_track_error)

        if abs(position_error) >= 0.30:
            self.KPt = 0.5
            self.KPp = 1.0
            self.max_angular_speed = 0.5 # radians per second

        elif abs(position_error) >= 0.10:
            self.KPt = 0.3
            self.KPp = 1.0
            self.max_angular_speed = 0.2 # radians per second
            if beta <= 0:
                position_error = position_error

        elif abs(position_error) >= 0.02:
            self.KPt = 0.05
            self.KPp = 0.5
            self.max_angular_speed = 0.05 # radians per second
        else:
            
            self.KPt = 0.0
            self.KPp = 0.0
            self.max_angular_speed = 0.05 # radians per second

        # Compute the orientation error
        orientation_error = theta_desired - yaw_actual

        # Normalize orientation_error to the range [-pi, pi]
        orientation_error = (orientation_error + math.pi) % (2 * math.pi) - math.pi

        # Integrate errors (Integral term)
        self.position_integral += position_error
        self.orientation_integral += orientation_error
        self.cross_track_integral += cross_track_error

        # Saturate integrals to prevent windup
        self.position_integral = max(min(self.position_integral, 1.0), -1.0)
        self.orientation_integral = max(min(self.orientation_integral, 1.0), -1.0)
        self.cross_track_integral = max(min(self.cross_track_integral, 1.0), -1.0)

        # Compute the control outputs
        linear_velocity_control_action = (
            self.KPp * position_error +
            self.KIp * self.position_integral
        )
        
        angular_velocity_control_action = (
            self.KPt * orientation_error +
            self.KPxte * cross_track_error +
            self.KIt * self.orientation_integral +
            self.KIxte * self.cross_track_integral
        )

        # Apply saturation limits to the control outputs
        linear_velocity_control_action = max(min(linear_velocity_control_action, self.max_linear_speed), -self.max_linear_speed)
        angular_velocity_control_action = max(min(angular_velocity_control_action, self.max_angular_speed), -self.max_angular_speed)
        
        return linear_velocity_control_action, angular_velocity_control_action

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
