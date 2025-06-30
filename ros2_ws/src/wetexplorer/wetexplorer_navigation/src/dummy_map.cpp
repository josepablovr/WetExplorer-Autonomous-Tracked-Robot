// src/dummy_map_rings_publisher.cpp

#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class DummyMapRingsPublisher : public rclcpp::Node
{
public:
  DummyMapRingsPublisher()
  : Node("dummy_map_rings_publisher")
  {
    // Declare and read the "ref" parameter, defaulting to "map"
    this->declare_parameter<std::string>("ref", "map");
    this->get_parameter("ref", ref_frame_);

    pub_ = this->create_publisher<MarkerArray>("/map_rings", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&DummyMapRingsPublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    MarkerArray arr;
    // Example points
    std::vector<std::pair<double,double>> pts = {
      {1.0,  1.0,
      {-1.0, 1.5}
    };

    for (size_t i = 0; i < pts.size(); ++i) {
      Marker m;
      m.header.frame_id = ref_frame_;
      m.header.stamp    = this->now();
      m.ns              = "goals";
      m.id              = static_cast<int>(i);
      m.type            = Marker::SPHERE;
      m.action          = Marker::ADD;
      m.pose.position.x = pts[i].first;
      m.pose.position.y = pts[i].second;
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.01;
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }

    pub_->publish(arr);
    RCLCPP_INFO(
      this->get_logger(),
      "Published %zu dummy goals on /map_rings (frame: %s)",
      arr.markers.size(),
      ref_frame_.c_str());
  }

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr            timer_;
  std::string                             ref_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyMapRingsPublisher>());
  rclcpp::shutdown();
  return 0;
}
