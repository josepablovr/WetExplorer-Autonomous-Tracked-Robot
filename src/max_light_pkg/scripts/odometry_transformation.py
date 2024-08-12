#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class GPSIMUTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gps_imu_transformer', anonymous=True)
        
        # Set up subscribers
        self.odom_sub = rospy.Subscriber('/odometry/gps', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Set up publisher
        self.corrected_odom_pub = rospy.Publisher('/odometry_gps_corrected', Odometry, queue_size=10)
        
        # Set up TF listener
        self.tf_listener = tf.TransformListener()
        
        # Store the latest data
        self.latest_odom = None
        self.latest_imu = None
    
    def odom_callback(self, msg):
        self.latest_odom = msg
        self.process_data()
    
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.process_data()
    
    def process_data(self):
        # Make sure we have received both odometry and IMU data
        if self.latest_odom is None or self.latest_imu is None:
            return
        
        try:
            # Get the transform from GPS frame to base_link
            self.tf_listener.waitForTransform("base_link", "gps_link", rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform("base_link", "gps_link", rospy.Time(0))
            
            # Convert the GPS position into a PointStamped
            gps_point = tf2_geometry_msgs.PointStamped()
            gps_point.header.frame_id = "gps_link"
            gps_point.header.stamp = rospy.Time(0)
            gps_point.point = self.latest_odom.pose.pose.position
            
            # Transform GPS position to base_link frame
            transformed_point = self.tf_listener.transformPoint("base_link", gps_point)
            
            # Prepare the corrected odometry message
            corrected_odom = Odometry()
            corrected_odom.header.stamp = rospy.Time.now()
            corrected_odom.header.frame_id = "base_link"  # Set the frame to base_link
            
            # Set the position (transformed GPS position)
            corrected_odom.pose.pose.position = transformed_point.point
            
            # Set the orientation from the IMU data
            corrected_odom.pose.pose.orientation = self.latest_imu.orientation
            
            # Optionally, you can also set the twist (velocity) from the original GPS odometry
            corrected_odom.twist.twist = self.latest_odom.twist.twist
            
            # Publish the corrected odometry
            self.corrected_odom_pub.publish(corrected_odom)
            
            #rospy.loginfo("Published corrected GPS odometry to /odometry_gps_corrected")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Exception: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = GPSIMUTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass
