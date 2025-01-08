import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import torch
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

class PointCloudServiceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_service_node')

        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/masked_depth_image',
            self.depth_image_callback,
            10
        )

        # Service
        self.service = self.create_service(
            Trigger,
            'generate_pointcloud_and_save_model',
            self.service_callback
        )

        # Utilities
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

        self.get_logger().info('PointCloudServiceNode is ready.')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.get_logger().info('Received camera info.')

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info('Received depth image.')

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
                z = self.depth_image[v, u] * 0.001  # Convert depth to meters
                if z > 0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def save_pointcloud(self, points):
        if points is None or len(points) == 0:
            self.get_logger().error('No points to save.')
            return

        # Save as PLY
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud("output.ply", cloud)
        self.get_logger().info('Point cloud saved as output.ply.')

    def save_model(self):
        # Dummy PyTorch model
        model = torch.nn.Linear(3, 1)
        torch.save(model.state_dict(), "model.pth")
        self.get_logger().info('Model saved as model.pth.')

    def service_callback(self, request, response):
        self.get_logger().info('Service called.')
        
        points = self.generate_pointcloud()
        if points is not None:
            self.save_pointcloud(points)
            self.save_model()
            response.success = True
            response.message = 'Point cloud and model generated and saved.'
        else:
            response.success = False
            response.message = 'Failed to generate point cloud or model.'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
