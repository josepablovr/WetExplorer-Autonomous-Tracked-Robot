import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import yaml
from cv_bridge import CvBridge

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Load parameters from config file
        self.declare_parameter('config_file', 'config/obstacle_params.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.load_config(config_file)

        # Subscribers
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/masked_depth_image',
            self.depth_image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        self.detection_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10)


        # Publisher for visualization marker
        self.marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.camera_model = None

        self.detection_array = None

    def load_config(self, config_file):
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            self.obstacle_radius = config.get('obstacle_radius', 0.5)
            self.obstacle_height = config.get('obstacle_height', 1.0)

    def camera_info_callback(self, msg):
        # Update camera model with the new camera info
        self.camera_model = msg

    def detection_callback(self, msg):
        # Store detection array for obstacle positioning
        self.detection_array = msg

    def depth_image_callback(self, msg):
        if self.camera_model is None or self.detection_array is None:
            # Wait until both camera info and detection data are received
            self.get_logger().info("Waiting for topics")
            return
        self.get_logger().info("Processing Obstacle")

        # Convert depth image to OpenCV format
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_image.shape

        # Get the depth values of detected obstacles
        for detection in self.detection_array.detections:
            bbox_center_x = float(detection.bbox.center.position.x)
            bbox_center_y = float(detection.bbox.center.position.y)

            # Convert coordinates to integers for array indexing
            bbox_center_x_int = int(bbox_center_x)
            bbox_center_y_int = int(bbox_center_y)

            # Extract depth points around the detection center
            points_depth = depth_image[
                max(0, bbox_center_y_int - 5): min(height, bbox_center_y_int + 5),
                max(0, bbox_center_x_int - 5): min(width, bbox_center_x_int + 5)
            ].flatten()

            # Calculate furthest 25% and closest 25% depth points
            valid_points = points_depth[points_depth > 0]  # Ignore zero depth points
            if len(valid_points) == 0:
                continue

            sorted_points = np.sort(valid_points)
            furthest_25 = sorted_points[-len(sorted_points) // 4:]
            closest_25 = sorted_points[:len(sorted_points) // 4]

            # Compute the middle point of the prism for depth
            mid_depth = (np.median(furthest_25) + np.median(closest_25)) / 2
            print(mid_depth)
            
            # Transform point from camera_link_output to base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'camera_link_output',
                    rclpy.time.Time())
                
                k = self.camera_model.k
                px, py, fx, fy = k[2], k[5], k[0], k[4]
                # Project from image to world space
                z = mid_depth/1000 # Convert from mm to meters if needed
                x =mid_depth*(bbox_center_x - px) / fx
                y = mid_depth*(bbox_center_y - py) / fy
                self.get_logger().info(f"X: {x}")
                self.get_logger().info(f"Y: {y}")
                self.get_logger().info(f"Z: {z}")
                point_camera = PointStamped()
                point_camera.header = msg.header
                point_camera.point.x = x
                point_camera.point.y = y
                point_camera.point.z = mid_depth

                point_base = tf2_geometry_msgs.do_transform_point(point_camera, transform)

                # Define the height for the obstacle
                obstacle_bottom = point_base.point.z - 0.25  # Distance from ground

                # Log detection message
                self.get_logger().info("Obstacle Detected")

                # Publish the 3D marker for visualization
                self.publish_marker(point_base.point.x, point_base.point.y, obstacle_bottom)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Failed to transform point: {e}')

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + self.obstacle_height / 2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = self.obstacle_radius * 2
        marker.scale.y = self.obstacle_radius * 2
        marker.scale.z = self.obstacle_height

        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        self.marker_pub.publish(marker)
        self.get_logger().info("Obstacle Detected")



def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)

    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
