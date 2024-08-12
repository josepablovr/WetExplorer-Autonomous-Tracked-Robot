#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf.transformations as tf_trans

class SensorOffsetCorrector:
    def __init__(self):
        rospy.init_node('heading_offset_corrector')     

        self.odom_sub = rospy.Subscriber('/odometry/gps', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu/gps_heading', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odometry/input', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/gps_heading_corrected', Imu,  queue_size=10)
        self.latest_imu_heading = None
        self.latest_odom = None
        self.lastest_imu_msg = None
        self.freq = 5 
        self.min_dt = 1/(5*self.freq)
        self.order_identified = True
        self.odom_time = None
        self.imu_time = None
        self.odom_first = False


    def imu_callback(self, msg):        
                   
        
        # Extract yaw from IMU quaternion
        orientation_q = msg.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        yaw_degrees = math.degrees(yaw) - 90
     
        yaw_rad = math.radians(yaw_degrees)
        self.latest_imu_heading = yaw_rad
        #self.publish_combined_odom()   
        self.lastest_imu_msg = msg
        #Correct imu angle
        corrected_imu = self.lastest_imu_msg
        new_orientation_q = tf_trans.quaternion_from_euler(0, 0, self.latest_imu_heading)

        
        corrected_imu.orientation.x = new_orientation_q[0]
        corrected_imu.orientation.y = new_orientation_q[1]
        corrected_imu.orientation.z = new_orientation_q[2]
        corrected_imu.orientation.w = new_orientation_q[3]
        self.imu_pub.publish(corrected_imu)


    def odom_callback(self, msg):
        self.latest_odom = msg
        self.publish_combined_odom()
        
    def publish_combined_odom(self):
        
        
        if self.latest_odom is not None:
            combined_odom = self.latest_odom
            # Get current orientation quaternion from odometry            
            accuracy = 0.05
            #combined_odom.header.frame_id = "odom"
            combined_odom.pose.covariance = [
                                    accuracy, 0, 0, 0, 0, 0,
                                    0, accuracy, 0, 0, 0, 0,
                                    0, 0, accuracy, 0, 0, 0,
                                    0, 0, 0, 100, 0, 0,
                                    0, 0, 0, 0, 100, 0,
                                    0, 0, 0, 0, 0, 100
                                        ]
            
             # Modify the twist covariance
            accuracy = 5
            combined_odom.twist.covariance = [
                                    accuracy, 0, 0, 0, 0, 0,
                                    0, accuracy, 0, 0, 0, 0,
                                    0, 0, accuracy, 0, 0, 0,
                                    0, 0, 0, 100, 0, 0,
                                    0, 0, 0, 0, 100, 0,
                                    0, 0, 0, 0, 0, 100
                                        ]


          
            self.odom_pub.publish(combined_odom)


            
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SensorOffsetCorrector()
    node.run()
