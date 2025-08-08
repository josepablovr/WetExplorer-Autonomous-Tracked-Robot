// src/joy_to_goal_node.cpp

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class JoyToGoalNode : public rclcpp::Node {
public:
  JoyToGoalNode()
  : Node("joy_to_goal_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare and read the "ref" parameter (either "odom" or "map")
    this->declare_parameter<std::string>("ref", "map");
    this->get_parameter("ref", ref_frame_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_teleop/joy", 10,
      std::bind(&JoyToGoalNode::joy_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Button index 3 corresponds to “button 4” on typical joysticks
    if (msg->buttons.size() > 3 && msg->buttons[3] == 1) {
      RCLCPP_INFO(this->get_logger(),
        "Button 4 pressed, computing object pose in '%s' frame.", ref_frame_.c_str());
      try {
        // 1) Transform from object -> camera_link
        auto tf_object_camera = tf_buffer_.lookupTransform(
          "camera_link", "object", tf2::TimePointZero);

        // Build a pose in camera_link frame
        geometry_msgs::msg::PoseStamped object_in_camera;
        object_in_camera.header.frame_id = "camera_link";
        object_in_camera.header.stamp = this->now();
        object_in_camera.pose.position.x = tf_object_camera.transform.translation.x;
        object_in_camera.pose.position.y = tf_object_camera.transform.translation.y;
        object_in_camera.pose.position.z = tf_object_camera.transform.translation.z;
        object_in_camera.pose.orientation = tf_object_camera.transform.rotation;

        // 2) Transform object pose into base_link
        auto object_in_base = tf_buffer_.transform(
          object_in_camera, "base_link");

        // 3) Finally transform from base_link into the desired reference frame
        auto object_in_ref = tf_buffer_.transform(
          object_in_base, ref_frame_);

        // Flatten to 2D: zero out z and reset orientation to upright
        object_in_ref.pose.position.z = 0.0;
        object_in_ref.pose.orientation = tf2::toMsg(
          tf2::Quaternion(0, 0, 0, 1)
        );

        // Publish the 2D goal
        goal_pub_->publish(object_in_ref);
        RCLCPP_INFO(this->get_logger(),
          "Published 2D goal in '%s': x=%.2f, y=%.2f",
          ref_frame_.c_str(),
          object_in_ref.pose.position.x,
          object_in_ref.pose.position.y
        );
      }
      catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
          "Failed to transform object into '%s': %s",
          ref_frame_.c_str(), ex.what());
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;

  std::string ref_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
