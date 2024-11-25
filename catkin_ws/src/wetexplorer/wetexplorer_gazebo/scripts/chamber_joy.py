#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

def map_range(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joy_callback(msg):
    # Extract the third element from the axes array (index 2)
    third_axis_value = msg.axes[2]

    # Check if the fifth or sixth button (index 4 or 5) is pressed
    button_5_pressed = msg.buttons[4] == 1
    button_6_pressed = msg.buttons[5] == 1

    if button_5_pressed or button_6_pressed:
        # Map the value from [1.0, -1.0] to [0.0, -0.19]
        mapped_value = map_range(third_axis_value, 1.0, -1.0, 0.0, -0.19)

        # Create the message to publish
        command_msg = Float64()
        command_msg.data = mapped_value

        # Publish the mapped value
        command_pub.publish(command_msg)

        #rospy.loginfo(f"Button pressed, Received {third_axis_value}, Mapped to {mapped_value}")
    """ else:
        rospy.loginfo("No relevant button pressed, command not published") """

def joy_to_linear_controller():
    rospy.init_node('joy_to_linear_controller', anonymous=True)

    # Subscribe to the /joy_teleop/joy topic
    rospy.Subscriber('/joy_teleop/joy', Joy, joy_callback)

    # Create a publisher for the /linear_controller/command topic
    global command_pub
    command_pub = rospy.Publisher('/linear_controller/command', Float64, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_to_linear_controller()
    except rospy.ROSInterruptException:
        pass
