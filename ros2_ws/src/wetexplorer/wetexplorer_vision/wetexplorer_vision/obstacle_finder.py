import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance, TransformStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, CameraInfo, Image
from vision_msgs.msg import Detection3D, BoundingBox3D, Detection2DArray
from yolo_msgs.msg import DetectionArray
from visualization_msgs.msg import Marker
import tf2_ros
import numpy as np
from cv_bridge import CvBridge

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscribers
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/masked_depth_image',
            self.depth_image_callback,
            10)

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detection/yolo/detections',
            self.detection_callback,
            10)

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Publisher for visualization marker
        self.marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)

        self.bridge = CvBridge()
        self.camera_model = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        # Update camera model with the new camera info
        self.camera_model = msg

    def depth_image_callback(self, msg):
        # Convert depth image to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_callback(self, msg):
        if self.camera_model is None or self.depth_image is None:
            self.get_logger().info("Waiting for Topics")
            return

        k = self.camera_model.k
        px, py, fx, fy = k[2], k[5], k[0], k[4]

        height, width = self.depth_image.shape
        self.get_logger().info("Processing Obstacle")
        for detection in msg.detections:
            # Process detections
            center_x = int(detection.bbox.center.position.x)
            center_y = int(detection.bbox.center.position.y)
            size_x = int(detection.bbox.size.x)
            size_y = int(detection.bbox.size.y)

            # Extract depth at the detection center
            depth_region = self.depth_image[
                max(0, center_y - 5): min(height, center_y + 5),
                max(0, center_x - 5): min(width, center_x + 5)
            ].flatten()

            # Filter out invalid depth points (zero or negative values)
            valid_depth_points = depth_region[depth_region > 0]
            if len(valid_depth_points) == 0:
                continue

            # Use median depth as the depth value
            z = np.median(valid_depth_points) / 1000.0  # Convert from mm to meters if needed
            print(z)
            z_min = z - 0.5  # Placeholder for minimum depth
            z_max = z + 0.5  # Placeholder for maximum depth

            # Project from image to world space
            x = z * (center_x - px) / fx
            y = z * (center_y - py) / fy
            w = z * (size_x / fx)
            h = z * (size_y / fy)

            # Create 3D BoundingBox
            bbox_3d = BoundingBox3D()
            bbox_3d.center.position.x = x
            bbox_3d.center.position.y = y
            bbox_3d.center.position.z = z
            bbox_3d.size.x = w
            bbox_3d.size.y = h
            bbox_3d.size.z = float(z_max - z_min)

            # Create and publish the marker for visualization
            self.publish_marker(bbox_3d)

    def publish_marker(self, bbox_3d):
        marker = Marker()
        marker.header.frame_id = 'camera_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = bbox_3d.center.position.x
        marker.pose.position.y = bbox_3d.center.position.y
        marker.pose.position.z = bbox_3d.center.position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = bbox_3d.size.x
        marker.scale.y = bbox_3d.size.y
        marker.scale.z = bbox_3d.size.z

        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)

    # Shutdown and destroy the node properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
