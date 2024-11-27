#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2

class RectangleDrawer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rectangle_drawer', anonymous=True)

        # Create a CvBridge instance
        self.bridge = CvBridge()

        # Subscribe to the RealSense camera image and camera info topics
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # Publisher for the modified image
        self.image_pub = rospy.Publisher('/camera/wetexplorer/rgb', Image, queue_size=10)

        # Store camera info
        self.camera_frame = None

    def camera_info_callback(self, camera_info):
        """Callback to get the camera frame from CameraInfo."""
        self.camera_frame = camera_info.header.frame_id

    def image_callback(self, data):
        rospy.logwarn("Proccess")
        if self.camera_frame is None:
            rospy.logwarn("No camera frame available yet. Waiting for CameraInfo.")
            return

        try:
            rospy.logwarn("Process2")
            # Convert the ROS Image message to a CV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Get image dimensions
            height, width, _ = cv_image.shape

            # Define the rectangle's top-left and bottom-right corners
            top_left = (width // 4, height // 4)
            bottom_right = (3 * width // 4, 3 * height // 4)

            # Draw a rectangle in the middle of the image (green color, thickness = 2)
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)

            # Create a new header with the camera_link frame
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.camera_frame

            # Convert the modified CV image back to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header = header  # Assign the frame to the image header

            # Publish the modified image
            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    try:
        RectangleDrawer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
