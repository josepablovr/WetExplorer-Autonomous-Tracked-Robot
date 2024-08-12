#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class ImuToBaseLink:
    def __init__(self):
        rospy.init_node('imu_to_base_link')
        
        self.imu_subscriber = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)
        # Offset in degrees for roll, pitch, and yaw
        self.offset_roll_degrees = 0
        self.offset_pitch_degrees = 180
        self.offset_yaw_degrees = 0

    def imu_callback(self, msg):
        try:
            # Lookup the transformation from imu_tf to base_link
            (trans, rot) = self.listener.lookupTransform('imu_tf', 'base_link', rospy.Time(0))
            
            # Create a quaternion from the IMU orientation message
             # Convert quaternion to Euler angles (roll, pitch, yaw)
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Apply offsets in degrees (convert offsets from degrees to radians)
            offset_roll_rad = math.radians(self.offset_roll_degrees)
            offset_pitch_rad = math.radians(self.offset_pitch_degrees)
            offset_yaw_rad = math.radians(self.offset_yaw_degrees)
            #print(np.degrees(roll), np.degrees(pitch), np.degrees(yaw))
            pitch = -pitch
             # Add offsets to Euler angles
            roll += offset_roll_rad
            pitch += offset_pitch_rad
            yaw += offset_yaw_rad
            
            # Convert Euler angles back to quaternion
            offset_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)


            # Convert the quaternion to a rotation matrix
            imu_rot_mat = tf.transformations.quaternion_matrix(offset_quaternion)

            

            # Apply the transformation from imu_tf to base_link to the IMU orientation
            
            base_link_rot_mat = tf.transformations.quaternion_matrix(rot)
           
            #print(base_link_rot_mat)
            world_rot_mat = np.dot(base_link_rot_mat, imu_rot_mat)
            #world_rot_mat = imu_rot_mat
            # Convert the combined rotation matrix back to quaternion
            world_quat = tf.transformations.quaternion_from_matrix(world_rot_mat)

            # Create the TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "base_link"

            # Set translation (assuming base_link is at the origin of the world frame)
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 1

            # Set rotation from the combined quaternion
            t.transform.rotation.x = world_quat[0]
            t.transform.rotation.y = world_quat[1]
            t.transform.rotation.z = world_quat[2]
            t.transform.rotation.w = world_quat[3]

            # Broadcast the transformation
            self.br.sendTransform(t)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ImuToBaseLink()
    node.run()
