# src/pose_publisher_action.py
#!/usr/bin/env python3
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

from wetexplorer_navigation.action import LocalizeObject

# your libs
from .registration_3d import Predator
from .pose_stats import Transform_Statistics


def rotmat_to_quat(R):
    # robust 3x3 -> [x,y,z,w], normalized
    q = np.zeros(4, dtype=np.float64)
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2,1] - R[1,2]) * s
        q[1] = (R[0,2] - R[2,0]) * s
        q[2] = (R[1,0] - R[0,1]) * s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            q[3] = (R[2,1] - R[1,2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0,1] + R[1,0]) / s
            q[2] = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            q[3] = (R[0,2] - R[2,0]) / s
            q[0] = (R[0,1] + R[1,0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            q[3] = (R[1,0] - R[0,1]) / s
            q[0] = (R[0,2] + R[2,0]) / s
            q[1] = (R[1,2] + R[2,1]) / s
            q[2] = 0.25 * s
    q /= np.linalg.norm(q) + 1e-12
    return q  # x,y,z,w


class PosePublisherAction(Node):
    def __init__(self):
        super().__init__('pose_publisher_action')

        # ── Parameters
        self.declare_parameter('cloud_topic', '/object/depth_cloud')
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame',  'object')
        self.declare_parameter('stale_timeout', 0.5)        # s
        self.declare_parameter('cloud_scale', 10.0)         # multiply points by this
        self.declare_parameter('min_points', 200)           # minimum points required
        self.declare_parameter('max_iterations', 1)         # clustering samples

        self.parent_frame   = self.get_parameter('parent_frame').value
        self.child_frame    = self.get_parameter('child_frame').value
        self.cloud_topic    = self.get_parameter('cloud_topic').value
        self.stale_timeout  = float(self.get_parameter('stale_timeout').value)
        self.cloud_scale    = float(self.get_parameter('cloud_scale').value)
        self.max_iterations = int(self.get_parameter('max_iterations').value)

        self.sim    = self.get_parameter('use_sim_time').value
        
        if self.sim:
            self.workspace = "/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/"
        else:
            self.workspace = "/workspaces/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/"
        self.config_path = self.workspace + "config/indoor.yaml"
        # ── State
        self._goal_active = False
        self.predator_initialized = False
        self.camera_info_sub = None
        self.camera_info = None

        self._cloud_lock = threading.Lock()
        self.pointcloud = None          # np.ndarray (N,3) float32
        self.last_cloud_stamp = None    # builtin_interfaces/Time

        # ── TF and viz
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 1)
        self.transform_AVG = Transform_Statistics()

        # ── Subscriptions
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, best_effort_qos
        )

        self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_cb, best_effort_qos
        )

        # ── Action server
        self._action_server = ActionServer(
            self, LocalizeObject, 'localize_object',
            execute_callback=self.execute_localization,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel
        )

        # ── Lazy init Predator at 1 Hz
        self.timer = self.create_timer(1.0, self._registration_timer_cb)

        self.get_logger().info('pose_publisher_action ready.')

    # ───────────────────────────────── Cloud & camera ─────────────────────────
    def cloud_cb(self, msg: PointCloud2):
        # Fast path: use numpy API if available
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x','y','z'), skip_nans=True)
            xyz = np.asarray(pts, dtype=np.float32)
        except Exception:
            gen = pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
            flat = np.fromiter((c for p in gen for c in p), dtype=np.float32, count=-1)
            if flat.size % 3 != 0:
                self.get_logger().warn('PointCloud2 size not divisible by 3; dropping remainder.')
                flat = flat[: flat.size // 3 * 3]
            xyz = flat.reshape(-1, 3)

        if xyz.size == 0:
            return

        # scale (param)
        if self.cloud_scale != 1.0:
            xyz *= self.cloud_scale

        with self._cloud_lock:
            self.pointcloud = xyz
            self.last_cloud_stamp = msg.header.stamp

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        # Pick parent frame from camera if present
        if msg.header.frame_id:
            self.parent_frame = msg.header.frame_id
        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None
            self.get_logger().info('Unsubscribed /camera/camera_info (got one).')

    # ───────────────────────────── Predator init timer ────────────────────────
    def _registration_timer_cb(self):
        if not self.predator_initialized:
            try:
                self.get_logger().info('[Timer] Initialising Predator …')
                self.registration = Predator(config_path = self.config_path, workspace=self.workspace)
                self.predator_initialized = bool(self.registration.initialize())
                self.get_logger().info(f"[Timer] Predator initialised: {self.predator_initialized}")

            except Exception as e:
                self.get_logger().error(f'Predator init failed: {e}')
                
        else:
            self.timer.cancel()

    # ─────────────────────────────── Action handlers ──────────────────────────
    def handle_goal(self, goal_request):
        if self._goal_active:
            self.get_logger().warn('Rejecting goal: previous localisation still executing.')
            return GoalResponse.REJECT

        # freshness check
        with self._cloud_lock:
            stamp = self.last_cloud_stamp

        if stamp is None:
            self.get_logger().warn('Rejecting goal: no point cloud yet.')
            return GoalResponse.REJECT

        now = self.get_clock().now()
        age = now - Time.from_msg(stamp)
        if age.nanoseconds < 0:
            age = Duration(seconds=0.0)

        if age > Duration(seconds=self.stale_timeout):
            self.get_logger().warn(
                f"Rejecting goal: cloud stale (age={age.nanoseconds/1e9:.3f}s > {self.stale_timeout:.3f}s)."
            )
            return GoalResponse.REJECT

        self._goal_active = True
        self.get_logger().info('Accepted localisation request.')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().info('Goal cancelled by client.')
        return CancelResponse.ACCEPT

    # ───────────────────────────── Execute localisation ───────────────────────
    def execute_localization(self, goal_handle):
        result = LocalizeObject.Result()
        try:
            if not self.predator_initialized:
                self.get_logger().info('Predator not ready; initialising (fallback)…')
                self.registration = Predator()
                self.predator_initialized = bool(self.registration.initialize())

            # snapshot pointcloud under lock
            with self._cloud_lock:
                cloud = None if self.pointcloud is None else self.pointcloud.copy()
                stamp = self.last_cloud_stamp

            if cloud is None or cloud.shape[0] < int(self.get_parameter('min_points').value):
                self.get_logger().warn('Aborting: cloud missing or too small.')
                goal_handle.abort()
                return result

            # collect a few transforms then cluster
            transforms, overlaps = [], []
            for i in range(self.max_iterations):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Cancel requested; stopping.')
                    goal_handle.canceled()
                    self._goal_active = False
                    return LocalizeObject.Result()

                self.get_logger().info(f'Running estimation {i+1}/{self.max_iterations}…')
                T, overlap = self.registration.run_Estimation(cloud)  # expects (N,3) np
                T = np.asarray(T, dtype=np.float32)     
                T[:3,3] /= self.cloud_scale
                transforms.append(T)
                overlaps.append(float(overlap))

            if len(transforms) == 0:
                self.get_logger().warn('No transforms estimated.')
                goal_handle.abort()
                return result

            mean_T = (self.transform_AVG.average_pose(transforms, overlaps)
                        if self.max_iterations >= 2 else transforms[0])

            t = mean_T[:3, 3]
            q = rotmat_to_quat(mean_T[:3, :3])  # x,y,z,w

            # publish TF
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.parent_frame
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = float(t[0])
            tf.transform.translation.y = float(t[1])
            tf.transform.translation.z = float(t[2])
            tf.transform.rotation.x = float(q[0])
            tf.transform.rotation.y = float(q[1])
            tf.transform.rotation.z = float(q[2])
            tf.transform.rotation.w = float(q[3])
            self.tf_broadcaster.sendTransform(tf)

            # marker
            marker = Marker()
            marker.header = tf.header
            marker.ns = 'transform_marker'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(t[0])
            marker.pose.position.y = float(t[1])
            marker.pose.position.z = float(t[2])
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.16
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            marker.lifetime = Duration(seconds=0.0).to_msg()
            self.marker_pub.publish(marker)

            # action result
            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position.x = float(t[0])
            pose.pose.position.y = float(t[1])
            pose.pose.position.z = float(t[2])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            result.pose = pose

            goal_handle.succeed()
            self._goal_active = False
            
            self.get_logger().info(f"Broadcasted TF {self.parent_frame} → {self.child_frame}")
            return result

        except Exception as e:
            self.get_logger().error(f'execute_localization error: {e}')
            goal_handle.abort()
            return result
        finally:
            self._goal_active = False

# ───────────────────────────────── Entry point ────────────────────────────────
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
