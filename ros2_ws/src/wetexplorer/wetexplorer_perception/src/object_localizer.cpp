// object_localizer.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <wetexplorer_navigation/action/localize_object.hpp>

#include <mutex>
#include <atomic>
#include <cmath>
#include <string>
#include <memory>

using LocalizeObject = wetexplorer_navigation::action::LocalizeObject;

class ObjectLocalizerNode : public rclcpp::Node {
public:
  explicit ObjectLocalizerNode()
  : Node("pose_publisher_action_centroid_cpp"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(*this)
  {
    // Parameters
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "camera1_link_output");
    child_frame_  = this->declare_parameter<std::string>("child_frame",  "object");
    cloud_timeout_sec_ = this->declare_parameter<double>("cloud_timeout_sec", 0.3);

    // QoS (BEST_EFFORT, KEEP_LAST 1)
    rclcpp::QoS best_effort_qos(1);
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    best_effort_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // Subscription
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object/depth_cloud", best_effort_qos,
      std::bind(&ObjectLocalizerNode::cloudCallback, this, std::placeholders::_1));

    // Marker publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Action server
    using std::placeholders::_1;
    using std::placeholders::_2;
    action_server_ = rclcpp_action::create_server<LocalizeObject>(
      this,
      "localize_object_light",
      std::bind(&ObjectLocalizerNode::handleGoal,      this, _1, _2),
      std::bind(&ObjectLocalizerNode::handleCancel,    this, _1),
      std::bind(&ObjectLocalizerNode::handleAccepted,  this, _1));

    RCLCPP_INFO(get_logger(), "Centroid-based “localize_object” action (C++) ready.");
  }

private:
  // --- Point cloud handling ---------------------------------------------------
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cloud_mtx_);
    latest_cloud_ = msg;  // keep only the newest
    last_cloud_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    have_cloud_time_ = true;
  }

  static bool computeMeanCentroid(
      const sensor_msgs::msg::PointCloud2 & cloud,
      double & cx, double & cy, double & cz, size_t & count)
  {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

    double sx = 0.0, sy = 0.0, sz = 0.0;
    count = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0f) {
        sx += x; sy += y; sz += z;
        ++count;
      }
    }

    if (count == 0) return false;
    cx = sx / static_cast<double>(count);
    cy = sy / static_cast<double>(count);
    cz = sz / static_cast<double>(count);
    return true;
  }

  // --- Action plumbing --------------------------------------------------------
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const LocalizeObject::Goal>)
  {
    // Only one goal at a time
    if (goal_active_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Goal rejected – previous one still running.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Freshness check
    {
      std::lock_guard<std::mutex> lk(cloud_mtx_);
      if (!latest_cloud_ || !have_cloud_time_) {
        RCLCPP_WARN(get_logger(), "Goal rejected – no point cloud received yet.");
        goal_active_ = false;
        return rclcpp_action::GoalResponse::REJECT;
      }
      const rclcpp::Time now = this->now();
      const rclcpp::Duration age = now - last_cloud_time_;
      if (age > rclcpp::Duration::from_seconds(cloud_timeout_sec_)) {
        RCLCPP_WARN(get_logger(),
          "Goal rejected – point cloud too old (age=%.3fs > %.3fs).",
          age.seconds(), cloud_timeout_sec_);
        goal_active_ = false;
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    RCLCPP_INFO(get_logger(), "Goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<LocalizeObject>>)
  {
    RCLCPP_INFO(get_logger(), "Goal cancelled by client.");
    goal_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<LocalizeObject>> goal_handle)
  {
    // Short task → execute inline
    execute(goal_handle);
  }

  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<LocalizeObject>> goal_handle)
  {
    auto result = std::make_shared<LocalizeObject::Result>();

    // 0) get the latest cloud --------------------------------------------------
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lk(cloud_mtx_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      RCLCPP_ERROR(get_logger(), "Point-cloud empty – aborting.");
      goal_handle->abort(result);
      goal_active_ = false;
      return;
    }

    // Optional: re-check freshness right before processing (race-safe)
    {
      const rclcpp::Time now = this->now();
      const rclcpp::Duration age = now - last_cloud_time_;
      if (age > rclcpp::Duration::from_seconds(cloud_timeout_sec_)) {
        RCLCPP_WARN(get_logger(),
          "Aborting – point cloud became stale (age=%.3fs > %.3fs).",
          age.seconds(), cloud_timeout_sec_);
        goal_handle->abort(result);
        goal_active_ = false;
        return;
      }
    }

    // 1) centroid --------------------------------------------------------------
    double cx=0, cy=0, cz=0; size_t n=0;
    if (!computeMeanCentroid(*cloud, cx, cy, cz, n)) {
      RCLCPP_ERROR(get_logger(), "No valid points in cloud – aborting.");
      goal_handle->abort(result);
      goal_active_ = false;
      return;
    }
    RCLCPP_INFO(get_logger(), "Centroid: [%.3f, %.3f, %.3f] m (N=%zu)", cx, cy, cz, n);

    // 2) orientation from base_link -------------------------------------------
    geometry_msgs::msg::TransformStamped tf_bl;
    bool have_tf = true;
    try {
      tf_bl = tf_buffer_.lookupTransform(
        parent_frame_, "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.3));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed – using identity rotation: %s", ex.what());
      have_tf = false;
    }

    // 3) broadcast TF ----------------------------------------------------------
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id  = child_frame_;
    tf_msg.transform.translation.x = cx;
    tf_msg.transform.translation.y = cy;
    tf_msg.transform.translation.z = cz;
    if (have_tf) {
      tf_msg.transform.rotation = tf_bl.transform.rotation;
    } else {
      tf_msg.transform.rotation.x = 0.0;
      tf_msg.transform.rotation.y = 0.0;
      tf_msg.transform.rotation.z = 0.0;
      tf_msg.transform.rotation.w = 1.0;
    }
    tf_broadcaster_.sendTransform(tf_msg);

    // 4) publish cylinder marker ----------------------------------------------
    visualization_msgs::msg::Marker marker;
    marker.header = tf_msg.header;
    marker.ns = "centroid_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz;
    marker.pose.orientation = tf_msg.transform.rotation;
    marker.scale.x = 0.35;  // diameter x
    marker.scale.y = 0.35;  // diameter y
    marker.scale.z = 0.16;  // height
    marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.3f;
    // marker.lifetime default (0) → forever

    marker_pub_->publish(marker);

    // 5) fill & return result --------------------------------------------------
    geometry_msgs::msg::PoseStamped pose_out;
    pose_out.header = tf_msg.header;
    pose_out.pose.position.x = cx;
    pose_out.pose.position.y = cy;
    pose_out.pose.position.z = cz;
    pose_out.pose.orientation = tf_msg.transform.rotation;

    result->pose = pose_out;
    goal_handle->succeed(result);
    goal_active_ = false;
  }

private:
  // params
  std::string parent_frame_;
  std::string child_frame_;
  double cloud_timeout_sec_{0.3};

  // IO
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // latest cloud + timing
  std::mutex cloud_mtx_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME}; // initialized; set in callback
  bool have_cloud_time_{false};

  // action
  std::atomic<bool> goal_active_{false};
  rclcpp_action::Server<LocalizeObject>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectLocalizerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
