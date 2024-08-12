#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import tf.transformations as tft

import numpy as np

class GPSTransformer:
    def __init__(self):
        rospy.init_node('gps_transformer')
        
        # Subscribing to odometry and IMU topics
        self.odom_sub = rospy.Subscriber('/odometry/gps', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu/data_filtered', Imu, self.imu_callback)
        
        # Publisher for the transformed odometry
        self.odom_pub = rospy.Publisher('/odometry/gps_data', Odometry, queue_size=10)
        
        self.imu_orientation = None

    def odom_callback(self, odom_msg):
        if self.imu_orientation is not None:
            # Extract position from the odometry message
            pos = odom_msg.pose.pose.position
            translation_world_to_base_link = np.array([pos.x, pos.y, pos.z])

            # Transformation parameters
            translation_gps_to_base_link = np.array([0, 0.55, -0.009])

            # Extract the quaternion from the IMU orientation
            x, y, z, w = self.imu_orientation.x, self.imu_orientation.y, self.imu_orientation.z, self.imu_orientation.w

            # Convert quaternion to roll, pitch, yaw
            # Roll (phi), pitch (theta), yaw (psi)
            roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
            pitch = np.arcsin(2 * (w * y - z * x))
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))

            # Construct rotation matrices for roll and pitch
            R_roll = np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]
            ])

            R_pitch = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ])

            # Since yaw is already considered in pos, we skip adding it
            R_world_to_base_link = R_pitch @ R_roll

            # Combine rotation and translation to form the homogeneous transformation matrix
            T_world_to_base_link = np.eye(4)
            T_world_to_base_link[:3, :3] = R_world_to_base_link
            T_world_to_base_link[:3, 3] = translation_world_to_base_link

            # Transformation matrix from gps to base_link
            T_gps_to_base_link = np.eye(4)
            T_gps_to_base_link[:3, 3] = translation_gps_to_base_link

            # Compute the inverse of T_gps_to_base_link
            T_gps_to_base_link_inv = np.eye(4)
            T_gps_to_base_link_inv[:3, 3] = translation_gps_to_base_link

            # Compute the final transformation matrix from world to gps
            T_world_to_gps = T_world_to_base_link @ T_gps_to_base_link_inv

            # Extract the position of base_link with respect to the world frame
            position_base_link_wrt_world = T_world_to_gps[:3, 3]

            transformed_position = position_base_link_wrt_world
            
            # Create a new odometry message
            new_odom = Odometry()
            new_odom.header = odom_msg.header
            new_odom.child_frame_id = odom_msg.child_frame_id
            
            new_odom.pose.pose.position.x = transformed_position[0]
            new_odom.pose.pose.position.y = transformed_position[1]
            new_odom.pose.pose.position.z = transformed_position[2]
                        
            
       
            
            # Publish the transformed odometry
            self.odom_pub.publish(new_odom)
    
    def imu_callback(self, imu_msg):
        # Store the latest IMU orientation
        self.imu_orientation = imu_msg.orientation

if __name__ == '__main__':
    try:
        transformer = GPSTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
