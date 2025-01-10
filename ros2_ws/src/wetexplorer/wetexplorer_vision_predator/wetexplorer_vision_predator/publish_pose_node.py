import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from .registration_predator import Predator  # Import the function
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import CameraInfo, Image
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.declare_parameter('parent_frame', 'camera_link')  # Parent frame
        self.declare_parameter('child_frame', 'object')  # Child frame
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # Initialize a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        

        self.registration = Predator()

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Specify the queue size
        )


        self.camera_callback_group = MutuallyExclusiveCallbackGroup()
        self.publish_callback_group = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            qos_profile=best_effort_qos,
            callback_group=self.camera_callback_group
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/masked_depth_image',
            self.depth_image_callback,
            qos_profile=best_effort_qos,
            callback_group=self.
            camera_callback_group
        )


        # self.timer = self.create_timer(
        #     1,  # Publish at 10 Hz
        #     self.publish_transform,
        #     callback_group=self.camera_callback_group
        # )


        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.get_logger().info('Received Camera info.')
        self.publish_transform()

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info('Received Depth image.')

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
                z = self.depth_image[v, u] * 0.01  # Convert depth to meters
                if z > 0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        return np.array(points, dtype=np.float32)
    
    def publish_transform(self):
        try:
            # Call the Predate_Pose function to get the transformation matrix
            
            points = self.generate_pointcloud()
            transformation_matrix = self.registration.run_Estimation(points)

            # Ensure it's a NumPy array and divide translation by 10 if needed
            transformation_matrix = np.array(transformation_matrix).copy()
            transformation_matrix[:3, 3] /= 10

            # Extract translation and rotation
            translation = transformation_matrix[:3, 3]
            rotation_matrix = transformation_matrix[:3, :3]

            # Convert rotation matrix to quaternion
            quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

            # Create a TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.parent_frame
            transform.child_frame_id = self.child_frame
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            # Publish the transform
            self.tf_broadcaster.sendTransform(transform)

            self.get_logger().info(f"Published transform from {self.parent_frame} to {self.child_frame}")
        except Exception as e:
            self.get_logger().error(f"Error publishing transform: {e}")

    @staticmethod
    def rotation_matrix_to_quaternion(rot_matrix):
        """
        Converts a 3x3 rotation matrix to a quaternion.
        """
        q = np.zeros(4)
        trace = np.trace(rot_matrix)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
            q[1] = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
            q[2] = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
        else:
            if rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
                q[3] = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
                q[2] = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
            elif rot_matrix[1, 1] > rot_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
                q[3] = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
                q[0] = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
                q[3] = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
                q[0] = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
                q[1] = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
                q[2] = 0.25 * s
        return q

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
