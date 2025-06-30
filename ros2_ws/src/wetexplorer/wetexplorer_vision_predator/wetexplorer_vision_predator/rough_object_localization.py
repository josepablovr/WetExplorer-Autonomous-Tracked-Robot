#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────────────
#  PosePublisherAction – centroid-based localisation (cylinder base
#  parallel to base_link)
# ──────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node         import Node
from rclpy.action       import ActionServer, GoalResponse, CancelResponse
from rclpy.qos          import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration     import Duration

from tf2_ros            import (TransformBroadcaster, Buffer, TransformListener,
                                LookupException, ConnectivityException,
                                ExtrapolationException)
from geometry_msgs.msg  import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import CameraInfo, Image
from cv_bridge          import CvBridge
import numpy as np

from wetexplorer_navigation.action import LocalizeObject


# ──────────────────────────────────────────────────────────────────────
class PosePublisherAction(Node):
    def __init__(self):
        super().__init__('pose_publisher_action_centroid')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('parent_frame', 'camera1_link_output')
        self.declare_parameter('child_frame',  'object')
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value

        # ── TF broadcaster + listener (for base_link orientation) ─────
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── mutex for Action goals ────────────────────────────────────
        self._goal_active = False      # only one goal at a time

        # ── QoS & I/O setup ───────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )


       

        self.camera_info  = None
        self.depth_image  = None
        self.pointcloud   = None
        self.bridge       = CvBridge()
        self.scale = 1

        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.camera_info_callback, qos_profile=best_effort_qos)

        self.create_subscription(
            Image, '/camera/masked_depth',
            self.depth_image_callback, qos_profile=best_effort_qos)

        self.marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)

        # ── Action server ─────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            LocalizeObject,
            'localize_object_light',
            execute_callback=self.execute_localization,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
        )

        self.get_logger().info('Centroid-based “localize_object” action ready.')

    # ──────────────────────────────────────────────────────────────
    #  Action callbacks
    # ──────────────────────────────────────────────────────────────
    def handle_goal(self, _goal_request):
        if self._goal_active:
            self.get_logger().warn('Goal rejected – previous one still running.')
            return GoalResponse.REJECT
        self._goal_active = True
        self.get_logger().info('Goal accepted.')
        return GoalResponse.ACCEPT

    def handle_cancel(self, _goal_handle):
        self.get_logger().info('Goal cancelled by client.')
        self._goal_active = False
        return CancelResponse.ACCEPT

    # ──────────────────────────────────────────────────────────────
    #  Topic callbacks
    # ──────────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        if msg.header.frame_id != '' and msg.header.frame_id != "camera1_link_output":
            self.parent_frame = msg.header.frame_id
            self.scale = 0.001
        else:
            self.parent_frame = "camera1_link_output"
            self.scale = 1

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.pointcloud  = self.generate_pointcloud()
      

    # ──────────────────────────────────────────────────────────────
    #  Point-cloud generator
    # ──────────────────────────────────────────────────────────────
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

    # ──────────────────────────────────────────────────────────────
    #  Robust centroid: median → 70 % trimmed mean
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def robust_centroid(pc):
        med = np.median(pc, axis=0)
        d   = np.linalg.norm(pc - med, axis=1)
        keep = d < np.percentile(d, 70)
        return pc[keep].mean(axis=0) if keep.any() else med

    # ──────────────────────────────────────────────────────────────
    #  Action execution
    # ──────────────────────────────────────────────────────────────
    def execute_localization(self, goal_handle):
        result = LocalizeObject.Result()

        # 0)  sanity ----------------------------------------------------
        if self.pointcloud is None or self.pointcloud.size == 0:
            self.get_logger().error('Point-cloud empty – aborting.')
            goal_handle.abort()
            self._goal_active = False
            return result

        # 1)  centroid --------------------------------------------------
        #centroid = self.robust_centroid(self.pointcloud)
        centroid = self.pointcloud.mean(axis=0) 
        self.get_logger().info(
            f"Robust centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}] m")

        # 2)  orientation from base_link -------------------------------
        try:
            tf_bl = self.tf_buffer.lookup_transform(
                self.parent_frame,        # target frame
                'base_link',              # source frame
                rclpy.time.Time(),        # latest
                timeout=rclpy.duration.Duration(seconds=0.3))
            rot = tf_bl.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF lookup failed – using identity rotation.')
            quat = [0.0, 0.0, 0.0, 1.0]

        # 3)  broadcast TF ---------------------------------------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id  = self.child_frame
        tf_msg.transform.translation.x = float(centroid[0])
        tf_msg.transform.translation.y = float(centroid[1])
        tf_msg.transform.translation.z = float(centroid[2])
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, \
        tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = quat
        self.tf_broadcaster.sendTransform(tf_msg)

        # 4)  publish cylinder marker ----------------------------------
        marker = Marker()
        marker.header          = tf_msg.header
        marker.ns              = 'centroid_marker'
        marker.id              = 0
        marker.type            = Marker.CYLINDER
        marker.action          = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        marker.pose.orientation.x, marker.pose.orientation.y, \
        marker.pose.orientation.z, marker.pose.orientation.w = quat
        marker.scale.x = marker.scale.y = 0.30   # diameter 30 cm
        marker.scale.z = 0.16                    # height   16 cm
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 0.3)
        marker.lifetime = Duration(seconds=0).to_msg()
        self.marker_pub.publish(marker)

        # 5)  fill & return result -------------------------------------
        pose_out = PoseStamped()
        pose_out.header        = tf_msg.header
        pose_out.pose.position.x = centroid[0]
        pose_out.pose.position.y = centroid[1]
        pose_out.pose.position.z = centroid[2]
        pose_out.pose.orientation.x, pose_out.pose.orientation.y, \
        pose_out.pose.orientation.z, pose_out.pose.orientation.w = quat

        result.pose = pose_out
        goal_handle.succeed()
        self._goal_active = False
        return result


# ──────────────────────────────────────────────────────────────────────
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
