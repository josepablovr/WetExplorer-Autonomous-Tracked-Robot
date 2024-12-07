import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
import tf2_ros

class RobotSimLocalization(Node):
    def __init__(self):
        super().__init__('robot_sim_localization')
        
        # Subscriber to the TF topic
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/simulation/wetexplorer_tf',
            self.tf_callback,
            10)
        
        # Subscriber to the CameraInfo topic
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # Publisher for filtered odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/filtered', 10)
        
        # TF broadcaster for map to base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.current_transform = None
        self.last_camera_info_stamp = None

        # Timer to publish the map to base_link transform periodically
        self.timer = self.create_timer(0.02, self.publish_map_to_base_link_tf)

    def camera_info_callback(self, msg):
        # Store the timestamp from the CameraInfo message
        self.last_camera_info_stamp = msg.header.stamp

    def tf_callback(self, msg):
        # Extract the first transform from the TFMessage (assuming it contains only one transform)
        if not msg.transforms or self.last_camera_info_stamp is None:
            return
        
        transform = msg.transforms[0]
        self.current_transform = transform

        # Create an Odometry message based on the transform
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.last_camera_info_stamp  # Use the timestamp from the CameraInfo topic
        odom.header.frame_id = 'map'
        
        # Set the pose based on transform
        odom.pose = PoseWithCovariance()
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        
        odom.pose.pose.orientation = transform.transform.rotation
        
        # Setting covariance to zero for simplicity (you may adjust it as necessary)
        odom.pose.covariance = [0.0] * 36
        
        # Set the twist (assuming no velocity information from transform)
        odom.twist = TwistWithCovariance()
        odom.twist.covariance = [0.0] * 36
        
        # Publish the odometry message
        self.odom_publisher.publish(odom)
        
    def publish_map_to_base_link_tf(self):
        if self.current_transform is None or self.last_camera_info_stamp is None:
            return
        
        # Create and publish the transform from map to base_link
        map_to_base_link = TransformStamped()
        map_to_base_link.header.stamp = self.last_camera_info_stamp  # Use the timestamp from the CameraInfo topic
        map_to_base_link.header.frame_id = 'map'
        map_to_base_link.child_frame_id = 'base_link'
        map_to_base_link.transform.translation = self.current_transform.transform.translation
        map_to_base_link.transform.rotation = self.current_transform.transform.rotation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(map_to_base_link)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimLocalization()
    rclpy.spin(node)
    
    # Shutdown and destroy the node properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
