import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from yolo_msgs.msg import Detection, DetectionArray, BoundingBox3D
import numpy as np
import cv2
import os
import open3d as o3d
from std_srvs.srv import Trigger

class MaskedImagePublisher(Node):
    def __init__(self):
        super().__init__('masked_image_publisher')
        
        # Subscribers
        self.create_subscription(DetectionArray, '/yolo/detections', self.detection_callback, 10)
        # self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        # self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        # self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.masked_depth_pub = self.create_publisher(Image, '/camera/masked_depth', 10)
        self.masked_color_pub = self.create_publisher(Image, '/camera/masked_color', 10)
        self.mask_pub = self.create_publisher(Image, '/camera/mask', 10)

        # Service  
        self.service = self.create_service(
            Trigger,
            'save_data',
            self.save_data_service
        )
        # Utility variables
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.camera_info = None
        self.mask = None
        self.counter = 1
        self.masked_depth = None
        

    def camera_info_callback(self, msg):
        
        #self.get_logger().info('Camera info Callback.')
        
        self.camera_info = msg

    def depth_callback(self, msg):
        #self.get_logger().info('Depth Callback.')
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        #self.get_logger().info('Color Callback.')
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg: DetectionArray):
        #self.get_logger().info('Detection Callback.')
        if self.depth_image is None or self.color_image is None or self.camera_info is None:
            #self.get_logger().info('Some topic(s) are missing...')
            return

        for detection in msg.detections:
            self.process_detection(detection)

    def process_detection(self, detection: Detection):
        center_x = int(detection.bbox.center.position.x)
        center_y = int(detection.bbox.center.position.y)
        size_x = int(detection.bbox.size.x)
        size_y = int(detection.bbox.size.y)
        #node.get_logger().info('Processing Detection.')

        if detection.mask.data:
            # Create mask from detection
            mask_array = np.array(
                [[int(ele.x), int(ele.y)] for ele in detection.mask.data]
            )
            self.mask = np.zeros(self.depth_image.shape[:2], dtype=np.uint8)
            cv2.fillPoly(self.mask, [np.array(mask_array, dtype=np.int32)], 255)

            # Mask depth and color images
            self.masked_depth = cv2.bitwise_and(self.depth_image, self.depth_image, mask=self.mask)
            masked_color = cv2.bitwise_and(self.color_image, self.color_image, mask=self.mask)

            # Publish masked images
            if (detection.class_name == "ring" and detection.score >= 0.9 and detection.bbox.size.x * detection.bbox.size.y > 7500):
                self.get_logger().error('Proper Ring detected')
                self.publish_masked_images(self.masked_depth, masked_color, self.mask)

    def publish_masked_images(self, masked_depth, masked_color, mask):
        depth_msg = self.bridge.cv2_to_imgmsg(masked_depth, encoding='passthrough')
        color_msg = self.bridge.cv2_to_imgmsg(masked_color, encoding='bgr8')
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')

        self.masked_depth_pub.publish(depth_msg)
        self.masked_color_pub.publish(color_msg)
        self.mask_pub.publish(mask_msg)

        self.get_logger().info('Published masked images and mask.')

    def generate_pointcloud(self):
        if self.camera_info is None or self.depth_image is None:
            self.get_logger().error('Missing camera info or depth image.')
            return None

        # Intrinsic camera matrix
        K = np.array(self.camera_info.k).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        # Generate point cloud
        points = []
        h, w = self.depth_image.shape
        for v in range(h):
            for u in range(w):
                z = self.masked_depth[v, u] * 0.01  # Convert depth to meters
                if z > 0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def save_pointcloud(self, points, index):
        if points is None or len(points) == 0:
            self.get_logger().error('No points to save.')
            return

        # Save as PLY without colors
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        directory = '/workspaces/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/train'
        file_path = os.path.join(directory, f'tgt_{index}.ply')
        while os.path.exists(file_path):
            index += 1
            file_path = os.path.join(directory, f'tgt_{index}.ply')
        o3d.io.write_point_cloud(file_path, cloud)

    def save_data_service(self, request, response):
        if self.depth_image is None or self.color_image is None or self.mask is None:
            response.success = False
            response.message = 'Missing required data for saving.'
            return response

        # Save pointcloud
        points = self.generate_pointcloud()
        self.save_pointcloud(points, self.counter)

        # Save images and mask
        directory = '/workspaces/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/train'
        m_depth_file = os.path.join(directory, f'm_depth_{self.counter}.png')
        while os.path.exists(m_depth_file):
            self.counter += 1
            m_depth_file = os.path.join(directory, f'm_depth_{self.counter}.png')

        cv2.imwrite(m_depth_file, self.depth_image)
        cv2.imwrite(os.path.join(directory, f'm_color_{self.counter}.png'), self.color_image)
        cv2.imwrite(os.path.join(directory, f'mask_{self.counter}.png'), self.mask)

        # Save mask array
        np.savetxt(os.path.join(directory, f'mask_{self.counter}_array.txt'), self.mask, fmt='%d')

        response.success = True
        response.message = 'Data successfully saved.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MaskedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
