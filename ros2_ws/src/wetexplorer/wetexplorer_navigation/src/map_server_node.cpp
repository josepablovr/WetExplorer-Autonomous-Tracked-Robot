#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <array>
#include <cmath>
#include <random>

struct Ring {
  int id;
  geometry_msgs::msg::PoseStamped pose;
  std::array<float, 3> color;
};

class MapServer : public rclcpp::Node {
public:
  MapServer()
  : Node("map_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    gen_(rd_()),
    dist_(0.0, 1.0)
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("base_frame", "camera1_link_output");
    this->declare_parameter<std::string>("map_frame", "odom");
    this->declare_parameter<std::string>("input_topic", "/visualization_marker");
    this->declare_parameter<std::string>("output_topic", "/map_rings");
    this->declare_parameter<double>("merge_threshold", 0.5);
    this->declare_parameter<double>("ring_diameter", 0.30);
    this->declare_parameter<double>("ring_height", 0.16);

    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("merge_threshold", threshold_);
    this->get_parameter("ring_diameter", diameter_);
    this->get_parameter("ring_height", height_);

    // Publisher and Subscriber
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic_, 10);
    sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      input_topic_, 10,
      std::bind(&MapServer::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "[map_server] Listening on '%s' (Marker), publishing array to '%s'",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void callback(const visualization_msgs::msg::Marker::SharedPtr m) {
    RCLCPP_INFO(this->get_logger(), "[map_server] Received marker id %d in frame '%s'",
                m->id, m->header.frame_id.c_str());

    // Build PoseStamped in base frame
    geometry_msgs::msg::PoseStamped ps_base;
    ps_base.header = m->header;
    ps_base.header.frame_id = base_frame_;
    ps_base.pose = m->pose;

    // Transform to map frame
    geometry_msgs::msg::PoseStamped ps_map;
    try {
      auto t = tf_buffer_.lookupTransform(
        map_frame_, ps_base.header.frame_id,
        rclcpp::Time(0), rclcpp::Duration(1, 0)
      );
      tf2::doTransform(ps_base, ps_map, t);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "[map_server] TF2 error: %s", ex.what());
      return;
    }

    double x = ps_map.pose.position.x;
    double y = ps_map.pose.position.y;
    RCLCPP_DEBUG(this->get_logger(), "[map_server] Transformed pose: (%.2f, %.2f)", x, y);

    // Merge or add
    bool matched = false;
    for (auto & ring : rings_) {
      double dx = ring.pose.pose.position.x - x;
      double dy = ring.pose.pose.position.y - y;
      double dist = std::hypot(dx, dy);
      if (dist < threshold_) {
        ring.pose = ps_map;
        matched = true;
        RCLCPP_INFO(this->get_logger(), "[map_server] Updated ring id %d at (%.2f, %.2f)",
                    ring.id, x, y);
        break;
      }
    }
    if (!matched) {
      Ring new_ring;
      new_ring.id = static_cast<int>(rings_.size());
      new_ring.pose = ps_map;
      new_ring.color = randomColor();
      rings_.push_back(new_ring);
      RCLCPP_INFO(this->get_logger(), "[map_server] Added new ring id %d at (%.2f, %.2f)",
                  new_ring.id, x, y);
    }

    // Publish consolidated markers
    visualization_msgs::msg::MarkerArray output;
    for (const auto & ring : rings_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "ring";
      marker.id = ring.id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = ring.pose.pose;
      marker.scale.x = diameter_;
      marker.scale.y = diameter_;
      marker.scale.z = height_;
      marker.color.r = ring.color[0];
      marker.color.g = ring.color[1];
      marker.color.b = ring.color[2];
      marker.color.a = 1.0;
      output.markers.push_back(marker);
    }
    pub_->publish(output);
    RCLCPP_INFO(this->get_logger(), "[map_server] Published %zu consolidated rings", output.markers.size());
  }

  std::array<float, 3> randomColor() {
    return {static_cast<float>(dist_(gen_)),
            static_cast<float>(dist_(gen_)),
            static_cast<float>(dist_(gen_))};
  }

  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string base_frame_, map_frame_;
  std::string input_topic_, output_topic_;
  double threshold_, diameter_, height_;

  // Ring storage
  std::vector<Ring> rings_;

  // Random color generator
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dist_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
