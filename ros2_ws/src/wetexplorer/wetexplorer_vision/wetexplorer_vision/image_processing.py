import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import copy

class RedToBlueHSVNode(Node):
    def __init__(self):
        super().__init__('red_to_blue_hsv_node')
        self.bridge = CvBridge()
        # Hue range to detect (0–30 on 0–180 scale)
        self.hue_min = 160
        self.hue_max = 180
        # Saturation threshold (0–255 scale)
        self.sat_thresh = 105
        # Hue offset to apply for blue shift
        self.hue_offset = -70

        # Subscriber to raw camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image/raw',
            self.image_callback,
            10)

        # Publishers for HSV channels
        self.hue_publisher = self.create_publisher(Image, 'camera/image/hue', 10)
        self.sat_publisher = self.create_publisher(Image, 'camera/image/saturation', 10)
        self.val_publisher = self.create_publisher(Image, 'camera/image/value', 10)
        # Publisher for post-processed images
        self.publisher = self.create_publisher(
            Image,
            'camera/image/postprocessed',
            10)

        self.get_logger().info(
            f'RedToBlueHSVNode started: hue_range=({self.hue_min}-{self.hue_max}), '
            f'sat_thresh={self.sat_thresh}, hue_offset={self.hue_offset}')

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV BGR
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')            
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        image = cv_image.copy() 
        image_original = cv_image.copy() 
        # Convert BGR to HSV and split channels
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)

        # Publish each HSV channel
        

        # Mask for pixels with hue in red range AND saturation above threshold
        mask = (h >= self.hue_min) & (h <= self.hue_max) & (s > self.sat_thresh)

        # Apply hue offset for masked pixels
        h_mod = h.astype(np.int32)
        h_mod[mask] = (h_mod[mask] + self.hue_offset) % 135
        h_mod = h_mod.astype(np.uint8)

        # Merge modified HSV and convert to BGR
        hsv_mod = cv2.merge([h_mod, s, v])

        try:
            hue_msg = self.bridge.cv2_to_imgmsg(h_mod, encoding='mono8')
            sat_msg = self.bridge.cv2_to_imgmsg(s, encoding='mono8')
            val_msg = self.bridge.cv2_to_imgmsg(h, encoding='mono8')
            for m in (hue_msg, sat_msg, val_msg):
                m.header = msg.header
            self.hue_publisher.publish(hue_msg)
            self.sat_publisher.publish(sat_msg)
            self.val_publisher.publish(val_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error (HSV publish): {e}')

        bgr_mod = cv2.cvtColor(hsv_mod, cv2.COLOR_HSV2RGB)

        # Create output: black background, only show modified pixels
        output = np.zeros_like(cv_image)
        output = cv2.cvtColor(image_original, cv2.COLOR_RGB2BGR)
        #output = image_original
        try:
            out_msg = self.bridge.cv2_to_imgmsg(bgr_mod, encoding='rgb8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error (postprocessed): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RedToBlueHSVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()