import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils


class RingDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ring_detector', anonymous=True)

        # Get ROS parameters
        self.model_path = rospy.get_param('~model', '/catkin_ws/src/wetexplorer/wetexplorer_vision/scripts/ring2.tflite')
        self.num_threads = rospy.get_param('~num_threads', 4)
        self.enable_edgetpu = rospy.get_param('~enable_edgetpu', False)

        # Initialize CvBridge
        self.bridge = CvBridge()
        print(self.model_path)
        

        # Publisher for processed images
        self.image_pub = rospy.Publisher('/camera/wetexplorer/rgb', Image, queue_size=1)

        # Initialize the object detection model
        base_options = core.BaseOptions(
            file_name='/catkin_ws/src/wetexplorer/wetexplorer_vision/scripts/ring2.tflite', use_coral=self.enable_edgetpu, num_threads=self.num_threads)
        detection_options = processor.DetectionOptions(
            max_results=10, score_threshold=0.5)
        options = vision.ObjectDetectorOptions(
            base_options=base_options, detection_options=detection_options)
        self.detector = vision.ObjectDetector.create_from_options(options)

        self.initialized = False
        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

        rospy.loginfo("Ring Detector Node initialized.")

    def image_callback(self, msg):
        
        try:
            # Convert ROS Image message to OpenCV format
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert the image from BGR to RGB as required by the TFLite model
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Create a TensorImage object from the RGB image
            input_tensor = vision.TensorImage.create_from_array(rgb_image)

            # Run object detection estimation using the model
            detection_result = self.detector.detect(input_tensor)

            # Draw keypoints and edges on the input image
            image = utils.visualize(image, detection_result)

            # Publish the processed image
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
        
             

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = RingDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ring Detector Node shutting down.")
