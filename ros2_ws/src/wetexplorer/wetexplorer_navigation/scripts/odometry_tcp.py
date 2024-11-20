#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations

class OdometryTcpPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odometry_tcp', anonymous=True)
        
        # Publisher for the chamber_link position in the odom frame
        self.pose_pub = rospy.Publisher('/odometry/tcp', PoseStamped, queue_size=10)
        
        # Subscriber for odometry data
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odometry_callback)

        # TF2 Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def odometry_callback(self, msg):
        try:
            # Extract the current pose of base_link in the odom frame from the odometry message
            base_link_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            base_link_orientation_q = msg.pose.pose.orientation

            # Get the transform between odom and base_link
            transform_odom_base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))

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

            # Extract x, y, and orientation (theta) of chamber_link in the odom frame
            x_chamber_in_odom = T_odom_chamber[0]
            y_chamber_in_odom = T_odom_chamber[1]
            theta_base = np.arctan2(R_odom_base_link[1, 0], R_odom_base_link[0, 0])  # Orientation of base_link in odom frame

            # Publish the pose of the chamber_link in the odom frame
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

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform odometry data: {e}")

    def compute_transform(self, R1, T1, R2, T2):
        R_combined = np.dot(R1, R2)
        T_combined = np.dot(R1, T2) + T1
        return R_combined, T_combined

    def run(self):
        # Spin to keep the node running
        rospy.spin()

if __name__ == '__main__':
    odometry_tcp_publisher = OdometryTcpPublisher()
    odometry_tcp_publisher.run()
