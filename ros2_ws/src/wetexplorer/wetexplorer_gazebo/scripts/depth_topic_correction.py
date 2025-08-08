import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class DepthRepublisherNode(Node):
    def __init__(self):
        super().__init__('depth_republisher_node')

        # Create a CvBridge object for image conversion
        self.bridge = CvBridge()

        # Subscriber to the original /camera/depth_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth_raw',
            self.depth_callback,
            10
        )

        # Publisher for the republished depth image with correct encoding
        self.publisher = self.create_publisher(
            Image,
            '/camera/depth_corrected',
            10
        )

        self.get_logger().info("Depth Republisher Node has been started.")

    def depth_callback(self, msg):
        """Callback function for depth image conversion."""
        try:
            # Convert the incoming 32FC1 depth image to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  
       
            # Replace inf values with 65535
            depth_image[np.isinf(depth_image)] = 10    
         

            
            corrected_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')

            # Preserve the original header and publish the corrected message
            corrected_msg.header = msg.header
            self.publisher.publish(corrected_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthRepublisherNode()
    rclpy.spin(node)  # Keep the node spinning to handle callbacks
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
