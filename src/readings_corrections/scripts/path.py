#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        self.path_discrete = Path()
        self.path_continuous = Path()

        self.path_gps = Path()
        

        self.path_discrete.header.frame_id = "world"
        self.path_continuous.header.frame_id = "world"
        self.path_gps.header.frame_id = "world"

        self.path_discrete_pub = rospy.Publisher('/path_discrete', Path, queue_size=1)
        self.path_continuous_pub = rospy.Publisher('/path_continuous', Path, queue_size=1)
        self.path_gps_pub = rospy.Publisher('/path_gps_input', Path, queue_size=1)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odometry_callback_1)
        rospy.Subscriber('/odometry/filtered_output', Odometry, self.odometry_callback_2)
        rospy.Subscriber('/odometry/gps', Odometry, self.odometry_callback_3)

    def odometry_callback_1(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append pose to discrete path
        self.path_discrete.poses.append(pose)        
  

        # Publish the paths
        self.path_discrete_pub.publish(self.path_discrete)
      

    def odometry_callback_2(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

      
        
        # Append pose to continuous path (this example is the same as discrete, you might want different handling)
        self.path_continuous.poses.append(pose)

       
        self.path_continuous_pub.publish(self.path_continuous)

    def odometry_callback_3(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

      
        
        # Append pose to continuous path (this example is the same as discrete, you might want different handling)
        self.path_gps.poses.append(pose)

       
        self.path_gps_pub.publish(self.path_gps)
if __name__ == '__main__':
    rospy.init_node('path_publisher', anonymous=True)
    path_publisher = PathPublisher()
    rospy.spin()
