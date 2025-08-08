#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

class OdomErrorNode : public rclcpp::Node
{
public:
  OdomErrorNode()
  : Node("odom_error_node"), global_received_(false)
  {
    // Subscribe to global odometry
    global_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/global", 10,
      std::bind(&OdomErrorNode::globalOdomCallback, this, std::placeholders::_1));

    // Subscribe to GPS odometry
    gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps", 10,
      std::bind(&OdomErrorNode::gpsOdomCallback, this, std::placeholders::_1));

    // Publisher for RMS error
    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("error", 10);

    RCLCPP_INFO(this->get_logger(), "OdomErrorNode initialized");
  }

private:
  void globalOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store the latest global pose
    global_pose_ = msg->pose.pose;
    global_received_ = true;
  }

  void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!global_received_) {
      RCLCPP_WARN(this->get_logger(), "Global odom not received yet. Skipping error calculation.");
      return;
    }

    // Compute differences
    double dx = msg->pose.pose.position.x - global_pose_.position.x;
    double dy = msg->pose.pose.position.y - global_pose_.position.y;
    double dz = msg->pose.pose.position.z - global_pose_.position.z;

    // Root Mean Square Error over x, y, z
    double rms = std::sqrt((dx*dx + dy*dy + dz*dz));

    // Publish error
    auto error_msg = std_msgs::msg::Float64();
    error_msg.data = rms;
    error_pub_->publish(error_msg);

    RCLCPP_INFO(this->get_logger(), "Published RMS error: %.6f", rms);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;

  geometry_msgs::msg::Pose global_pose_;
  bool global_received_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomErrorNode>());
  rclcpp::shutdown();
  return 0;
}