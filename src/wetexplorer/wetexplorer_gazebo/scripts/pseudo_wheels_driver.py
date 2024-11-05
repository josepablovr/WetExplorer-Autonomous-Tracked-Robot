#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class WheelVelocityController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('wheel_velocity_controller')

        # Create publishers for right wheels
        self.right_publishers = [
            rospy.Publisher(f'/wheel_{i}_right_velocity_controller/command', Float64, queue_size=10) 
            for i in range(1, 21)
        ]

        # Create publishers for left wheels
        self.left_publishers = [
            rospy.Publisher(f'/wheel_{i}_left_velocity_controller/command', Float64, queue_size=10) 
            for i in range(1, 21)
        ]

        # Subscribe to command topics for right and left wheels
        rospy.Subscriber('/command/pseudowheels_right', Float64, self.right_callback)
        rospy.Subscriber('/command/pseudowheels_left', Float64, self.left_callback)

    def right_callback(self, msg):
        """Callback function for right wheels command."""
        velocity = msg.data
        self.publish_velocity(velocity, self.right_publishers)

    def left_callback(self, msg):
        """Callback function for left wheels command."""
        velocity = msg.data
        self.publish_velocity(velocity, self.left_publishers)

    def publish_velocity(self, velocity, publishers):
        """Publishes the given velocity to all specified publishers."""
        for pub in publishers:
            pub.publish(velocity)

    def run(self):
        """Keeps the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = WheelVelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
