# src/pose_publisher_action.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
from .registration_predator import Predator        # your registration class
from .pose_evaluation import TransformClusterer    # your clustering helper
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
             # ➊   add

# Import the action type we just defined
from wetexplorer_navigation.action import LocalizeObject


class PosePublisherAction(Node):
    def __init__(self):
        super().__init__('pose_publisher_action')

        # Declare “ref”‐style parent/child frames (these can be overridden via ros2 param)
        self.declare_parameter('parent_frame', 'camera_color_optical_frame') #camera1_link_output
        self.declare_parameter('child_frame',  'object')
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value

        # TF broadcaster (we will still broadcast the transform to TF tree)
        self.tf_broadcaster = TransformBroadcaster(self)

        # We’ll only run up to 5 iterations to cluster transforms
        self.max_iterations = 5
        self.registration = None  # <— Predator instance will be lazily initialised

        # ──────────────────────────────────────────────────────────────
        # New: timer to lazily create Predator at 1 Hz if missing
        # ──────────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0, self._registration_timer_cb)  # 1 Hz

        # QoS for camera topics
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pointcloud = None
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

        # TF clustering helper
        self.clusterer = TransformClusterer(
            max_translation_tol=0.075,
            max_orientation_tol_deg=35.0,
            include_yaw=False
        )

        # Marker publisher (to visualize a cylinder at the final pose)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Subscriptions: camera info + masked depth
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile=best_effort_qos
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/masked_depth',
            self.depth_image_callback,
            qos_profile=best_effort_qos
        )

        # ActionServer: “localize_object”
        self._action_server = ActionServer(
            self,
            LocalizeObject,
            'localize_object',
            execute_callback=self.execute_localization,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel)
        

        self.get_logger().info('ActionServer “localize_object” is ready.')
        self.predator_initialized = False
       
        self._goal_active = False            #    flag: “goal in progress?”

        self.scale = 1
      
    # ──────────────────────────────────────────────────────────────
    # Timer callback
    # ──────────────────────────────────────────────────────────────
    def _registration_timer_cb(self):
        """Timer callback that ensures Predator is initialised.

        Runs at 1 Hz. If ``self.registration`` is still ``None`` we create the
        instance and run its ``initialize`` method. Once the registrator exists
        we stop the timer to save resources.
        """
        if not self.predator_initialized:
            self.get_logger().info('[Timer] Initialising Predator …')
            self.registration = Predator()
            self.predator_initialized = self.registration.initialize()
            self.get_logger().info('[Timer] Predator initialised.')
        else:
            # Predator already exists → cancel further timer calls
            self.timer.cancel()
        

    # ------------------------------------------------------------------
    # Action‑related callbacks
    # ------------------------------------------------------------------
    def handle_goal(self, goal_request):
        
        if self._goal_active:                      # a goal is still running
            self.get_logger().warn(
                'Rejecting new goal – previous localisation still executing.')
            return GoalResponse.REJECT
        self._goal_active = True                   # mark slot as taken
        self.get_logger().info('Accepted localisation request.')
        return GoalResponse.ACCEPT

    # ─── handle_cancel ───────────────────────────────────────────────────
    def handle_cancel(self, goal_handle):
        self.get_logger().info('Goal cancelled by client.')
       
        self._goal_active = False                  # free the slot
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        if msg.header.frame_id != '' and msg.header.frame_id != "camera1_link_output":
            self.parent_frame = msg.header.frame_id
            self.get_logger().info('Usim REAL camera')
            self.scale = 0.01 # Scale to cm
            self.max_translation_tol = 0.05
        else:
            self.get_logger().info('Usim SIM camera')
            self.parent_frame = "camera1_link_output"
            self.scale = 10
            self.max_translation_tol = 0.12

    def depth_image_callback(self, msg: Image):
        # Convert to CV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.pointcloud = self.generate_pointcloud()
        self.get_logger().info('Depth Info Updated.')
        

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------

    
    def generate_pointcloud(self):
        if self.camera_info is None or self.depth_image is None:
            self.get_logger().error('Missing camera info or depth image.')
            return None

        h, w = self.depth_image.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        z = self.depth_image * self.scale 
        x = (u - self.camera_info.k[2]) * z / self.camera_info.k[0]
        y = (v - self.camera_info.k[5]) * z / self.camera_info.k[4]

        mask = z > 0
        pointcloud = np.stack((x[mask], y[mask], z[mask]), axis=-1)
        return pointcloud

    # ------------------------------------------------------------------
    # Action execution
    # ------------------------------------------------------------------
    def execute_localization(self, goal_handle):
        """
        Called whenever a client sends a “localize_object” goal.
        Runs Predator, clusters transforms, publishes the final TF, marker,
        then returns the PoseStamped as the action result.
        """
        result = LocalizeObject.Result()

        # Ensure registration object exists (fallback safety‑check)
        if not self.registration.initialized:
            self.get_logger().info('Initializing Predator (fallback) …')
            self.registration = Predator()
            self.registration.initialize()
            self.get_logger().info('Initialized Predator registrator.')
        else:
            self.get_logger().info('Predator already initialized.')

        # We gather up to self.max_iterations sets of transforms, then cluster
        transform_matrices = []
        attempt = 0
        while True:
            
            for _ in range(self.max_iterations):
                self.get_logger().info('Running Estimation.')
                # run_Estimation returns a 4x4 matrix
                
                self.get_logger().info(f"Pointcloud has {self.pointcloud.shape[0]} points")
                T = self.registration.run_Estimation(self.pointcloud).copy()
                T = np.array(T)
                T[:3,3] /= 10.0  # scale translation if needed
                self.get_logger().info(f"Sample transform:\n{T}")
                transform_matrices.append(T)

            # Attempt clustering
            mean_T = self.clusterer.cluster_and_mean(transform_matrices)
            if mean_T is not None:
                self.get_logger().info(f"Mean transform found:\n{mean_T}")
                break
            else:
                self.get_logger().warn('No valid cluster; retrying…')
                transform_matrices.clear()
                attempt += 1
                if attempt > 3:
                    self.get_logger().error('Localization repeatedly failed.')
                    goal_handle.abort()
                    return result

        # Extract translation + rotation
        translation = mean_T[:3,  3]
        rot_mat     = mean_T[:3, :3]
        quaternion  = self.rotation_matrix_to_quaternion(rot_mat)

        # Broadcast TF: parent_frame → child_frame
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f"Broadcasted TF {self.parent_frame} → {self.child_frame}"
        )

        # Publish a cylinder marker at that pose (in parent_frame)
        marker = Marker()
        marker.header.stamp = tf_msg.header.stamp
        marker.header.frame_id = self.parent_frame
        marker.ns = 'transform_marker'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = translation[0]
        marker.pose.position.y = translation[1]
        marker.pose.position.z = translation[2]
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.16
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.marker_pub.publish(marker)
        self.get_logger().info("Published green cylinder marker.")

        # Fill action result (PoseStamped)
        pose_out = PoseStamped()
        pose_out.header.stamp = tf_msg.header.stamp
        pose_out.header.frame_id = self.parent_frame
        pose_out.pose.position.x = translation[0]
        pose_out.pose.position.y = translation[1]
        pose_out.pose.position.z = translation[2]
        pose_out.pose.orientation.x = quaternion[0]
        pose_out.pose.orientation.y = quaternion[1]
        pose_out.pose.orientation.z = quaternion[2]
        pose_out.pose.orientation.w = quaternion[3]

        result.pose = pose_out
        goal_handle.succeed()

        
                      
        self._goal_active = False
        return result

    # ------------------------------------------------------------------
    # Static helpers
    # ------------------------------------------------------------------
    @staticmethod
    def rotation_matrix_to_quaternion(rot_matrix):
        """Convert 3×3 rotation matrix to quaternion [x,y,z,w]."""
        q = np.zeros(4)
        trace = np.trace(rot_matrix)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (rot_matrix[2,1] - rot_matrix[1,2]) * s
            q[1] = (rot_matrix[0,2] - rot_matrix[2,0]) * s
            q[2] = (rot_matrix[1,0] - rot_matrix[0,1]) * s
        else:
            if rot_matrix[0,0] > rot_matrix[1,1] and rot_matrix[0,0] > rot_matrix[2,2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[0,0] - rot_matrix[1,1] - rot_matrix[2,2])
                q[3] = (rot_matrix[2,1] - rot_matrix[1,2]) / s
                q[0] = 0.25 * s
                q[1] = (rot_matrix[0,1] + rot_matrix[1,0]) / s
                q[2] = (rot_matrix[0,2] + rot_matrix[2,0]) / s
            elif rot_matrix[1,1] > rot_matrix[2,2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[1,1] - rot_matrix[0,0] - rot_matrix[2,2])
                q[3] = (rot_matrix[0,2] - rot_matrix[2,0]) / s
                q[0] = (rot_matrix[0,1] + rot_matrix[1,0]) / s
                q[1] = 0.25 * s
                q[2] = (rot_matrix[1,2] + rot_matrix[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[2,2] - rot_matrix[0,0] - rot_matrix[1,1])
                q[3] = (rot_matrix[1,0] - rot_matrix[0,1]) / s
                q[0] = (rot_matrix[0,2] + rot_matrix[2,0]) / s
                q[1] = (rot_matrix[1,2] + rot_matrix[2,1]) / s
                q[2] = 0.25 * s
        return q


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):    

    rclpy.init(args=args)
    node = PosePublisherAction()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
