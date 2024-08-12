#!/usr/bin/env python3
import rospy
if __name__ == '__main__':
    rospy.init_node("test_node")
    '''
    rospy.loginfo("Hello from test_node")
    rospy.logwarn("This is a Warning")
    rospy.logerr("This is an error")

    rospy.sleep(1.0)
    rospy.loginfo("End of program")
    '''
    rospy.loginfo("Test node has been started")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()