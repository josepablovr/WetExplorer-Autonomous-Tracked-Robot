// src/map_server_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <wetexplorer_navigation/action/update_map.hpp>
#include <wetexplorer_navigation/action/localize_object.hpp>

#include <vector>
#include <array>
#include <cmath>
#include <random>
#include <chrono>   // for std::chrono_literals

using namespace std::chrono_literals;

using UpdateMapAction   = wetexplorer_navigation::action::UpdateMap;
using LocalizeObjAction = wetexplorer_navigation::action::LocalizeObject;
using LocalGoalHandle   = rclcpp_action::ClientGoalHandle<LocalizeObjAction>;
using UpdateMapGoalHandle = rclcpp_action::ServerGoalHandle<UpdateMapAction>;

struct Ring
{
  int id;
  geometry_msgs::msg::PoseStamped pose;
  std::array<float, 3> color;
};

class MapServer : public rclcpp::Node
{
public:
  MapServer()
  : Node("map_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    gen_(rd_()),
    dist_(0.0, 1.5)
  {
    // ------------ Declare & Read Parameters ------------
    declare_parameter<std::string>("ref", "map");
    declare_parameter<std::string>("base_frame", "camera1_link_output");
    declare_parameter<std::string>("map_frame", "odom");
    declare_parameter<std::string>("input_topic", "/visualization_marker");
    declare_parameter<std::string>("output_topic", "/map_rings");
    declare_parameter<double>("merge_threshold", 1.0);
    declare_parameter<double>("ring_diameter", 0.35);
    declare_parameter<double>("ring_height", 0.16);

    get_parameter("ref",          ref_);
    get_parameter("base_frame",   base_frame_);
    get_parameter("map_frame",    map_frame_);
    get_parameter("input_topic",  input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("merge_threshold", threshold_);
    get_parameter("ring_diameter",   diameter_);
    get_parameter("ring_height",     height_);

    // If ref == "odom" or "map", override map_frame_ accordingly
    if (ref_ == "odom") {
      map_frame_ = "odom";
    } else if (ref_ == "map") {
      map_frame_ = "map";
    }

    RCLCPP_INFO(get_logger(),
      "[map_server] ref='%s', base_frame='%s', map_frame='%s',\n"
      "  listening on '%s', publishing to '%s' (1 Hz),\n"
      "  threshold=%.2f, diameter=%.2f, height=%.2f",
      ref_.c_str(), base_frame_.c_str(), map_frame_.c_str(),
      input_topic_.c_str(), output_topic_.c_str(),
      threshold_, diameter_, height_);

    // ------------ Publisher & Subscriber ------------
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, 10);

    sub_ = create_subscription<visualization_msgs::msg::Marker>(
      input_topic_, 10,
      std::bind(&MapServer::markerCallback, this, std::placeholders::_1)
    );

    // ------------ 1 Hz Timer for publishing consolidated map ------------
    timer_ = create_wall_timer(
      1s, std::bind(&MapServer::publishMap, this));

    // ------------ Action Client: localize_object ------------
    localize_client_ = rclcpp_action::create_client<LocalizeObjAction>(
      this, "localize_object");

    // ------------ Action Server: update_map ------------
    update_map_server_ = rclcpp_action::create_server<UpdateMapAction>(
      this,
      "update_map",
      std::bind(&MapServer::handleUpdateMapGoal,   this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MapServer::handleUpdateMapCancel, this, std::placeholders::_1),
      std::bind(&MapServer::executeUpdateMap,      this, std::placeholders::_1)
    );
  }

private:
  // ------------ Subscriber callback: integrate incoming Marker as ring(s) ------------
  void markerCallback(const visualization_msgs::msg::Marker::SharedPtr m)
  {
    RCLCPP_INFO(get_logger(),
      "[map_server] Received marker id %d in frame '%s'",
      m->id, m->header.frame_id.c_str());

    // Build a PoseStamped in base_frame_
    geometry_msgs::msg::PoseStamped ps_base;
    ps_base.header = m->header;
    ps_base.header.frame_id = base_frame_;
    ps_base.pose = m->pose;

    // Transform into map_frame_
    geometry_msgs::msg::PoseStamped ps_map;
    try {
      auto tf = tf_buffer_.lookupTransform(
        map_frame_,              // target
        ps_base.header.frame_id, // source
        rclcpp::Time(0),
        rclcpp::Duration(1, 0));
      tf2::doTransform(ps_base, ps_map, tf);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "[map_server] TF2 error: %s", ex.what());
      return;
    }

    double x = ps_map.pose.position.x;
    double y = ps_map.pose.position.y;

    // Merge if within threshold, else append new ring
    bool matched = false;
    for (auto & ring : rings_) {
      double dx = ring.pose.pose.position.x - x;
      double dy = ring.pose.pose.position.y - y;
      if (std::hypot(dx, dy) < threshold_) {
        ring.pose = ps_map;
        matched = true;
        RCLCPP_INFO(get_logger(),
          "[map_server] Updated ring id %d at (%.2f, %.2f)",
          ring.id, x, y);
        break;
      }
    }
    if (!matched) {
      Ring nr;
      nr.id    = static_cast<int>(rings_.size());
      nr.pose  = ps_map;
      nr.color = randomColor();
      rings_.push_back(nr);
      RCLCPP_INFO(get_logger(),
        "[map_server] Added new ring id %d at (%.2f, %.2f)",
        nr.id, x, y);
    }
    // (Publishing happens on the 1 Hz timer.)
  }

  // ------------ Timer callback (1 Hz): publish all rings as MarkerArray ------------
  void publishMap()
  {
    if (rings_.empty()) {
      return; // nothing to publish yet
    }

    visualization_msgs::msg::MarkerArray output;
    for (const auto & ring : rings_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp    = now();
      marker.ns              = "ring";
      marker.id              = ring.id;
      marker.type            = visualization_msgs::msg::Marker::CYLINDER;
      marker.action          = visualization_msgs::msg::Marker::ADD;
      marker.pose            = ring.pose.pose;
      marker.scale.x         = diameter_;
      marker.scale.y         = diameter_;
      marker.scale.z         = height_;
      marker.color.r         = ring.color[0];
      marker.color.g         = ring.color[1];
      marker.color.b         = ring.color[2];
      marker.color.a         = 0.6f;
      output.markers.push_back(marker);
    }
    
    pub_->publish(output);
    RCLCPP_DEBUG(get_logger(),
      "[map_server] Published %zu rings", output.markers.size());
  }

  // ======== ActionServer callbacks for UpdateMap ========

  // Accept any incoming update_map goal
  rclcpp_action::GoalResponse handleUpdateMapGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const UpdateMapAction::Goal> /*goal*/)
  {
    RCLCPP_INFO(get_logger(),
      "[update_map] Received new goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Accept cancel requests
  rclcpp_action::CancelResponse handleUpdateMapCancel(
    const std::shared_ptr<UpdateMapGoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(),
      "[update_map] Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // Execute the update_map action: calls localize_object and then returns all ring poses
  //--------------------------------------------------------------------
  void executeUpdateMap(
      const std::shared_ptr<UpdateMapGoalHandle> goal_handle)
  {
    // 1) Make sure localize_object server is available
    if (!localize_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(),
        "[update_map] localize_object server unavailable");
      goal_handle->abort(std::make_shared<UpdateMapAction::Result>());
      return;
    }

    // 2) Send an empty LocalizeObject goal
    auto local_goal = LocalizeObjAction::Goal();
    rclcpp_action::Client<LocalizeObjAction>::SendGoalOptions send_opts;
    send_opts.goal_response_callback = [](auto) { /* no-op */ };
    send_opts.feedback_callback      = [](auto, auto) { /* no-op */ };

    // 3) When we get the result …
    send_opts.result_callback =
      [this, goal_handle](const LocalGoalHandle::WrappedResult & wrapped)
      {
        //------------------------------------------------------------
        // A) Check action result
        //------------------------------------------------------------
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_ERROR(get_logger(),
            "[update_map] localize_object failed (code %d)",
            static_cast<int>(wrapped.code));
          goal_handle->abort(std::make_shared<UpdateMapAction::Result>());
          return;
        }
          
        //------------------------------------------------------------
        // B) Transform pose into map frame
        //------------------------------------------------------------
        geometry_msgs::msg::PoseStamped ps_src = wrapped.result->pose;  // original frame
        geometry_msgs::msg::PoseStamped ps_map;
        try {
          auto tf = tf_buffer_.lookupTransform(
            map_frame_,                    // target
            ps_src.header.frame_id,        // source
            rclcpp::Time(0),
            rclcpp::Duration(1, 0));
          tf2::doTransform(ps_src, ps_map, tf);
        }
        catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(get_logger(),
            "[update_map] TF2 error: %s", ex.what());
          goal_handle->abort(std::make_shared<UpdateMapAction::Result>());
          return;
        }

        double x = ps_map.pose.position.x;
        double y = ps_map.pose.position.y;

        //------------------------------------------------------------
        // C) Merge into rings_ (all stored in map frame)
        //------------------------------------------------------------
        bool matched = false;
        for (auto & ring : rings_) {
          double dx = ring.pose.pose.position.x - x;
          double dy = ring.pose.pose.position.y - y;
          if (std::hypot(dx, dy) < threshold_) {
            ring.pose = ps_map;                          // update + keep frame
            matched = true;
            RCLCPP_INFO(get_logger(),
              "[update_map] Updated ring id %d at (%.2f, %.2f) in %s",
              ring.id, x, y, map_frame_.c_str());
            break;
          }
        }

        if (!matched) {
          Ring nr;
          nr.id    = static_cast<int>(rings_.size());
          nr.pose  = ps_map;
          nr.color = randomColor();
          rings_.push_back(nr);
          RCLCPP_INFO(get_logger(),
            "[update_map] Added new ring id %d at (%.2f, %.2f) in %s",
            nr.id, x, y, map_frame_.c_str());
        }

        //------------------------------------------------------------
        // D) Rebuild ring_positions_ & recompute TSP
        //------------------------------------------------------------
        ring_positions_.clear();
        for (auto & r : rings_) {
          ring_positions_.emplace_back(
            r.pose.pose.position.x,
            r.pose.pose.position.y);
        }
        best_path_ = findTSP(ring_positions_);

        //------------------------------------------------------------
        // E) Fill & return UpdateMap result (poses already in map frame)
        //------------------------------------------------------------
        auto result = std::make_shared<UpdateMapAction::Result>();
        for (auto & r : rings_) {
          result->poses.push_back(r.pose);
        }
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(),
          "[update_map] Succeeded → returning %zu poses",
          result->poses.size());
      };

    // 4) Kick off the request
    localize_client_->async_send_goal(local_goal, send_opts);
  }


  // ================= Helpers =================
  std::array<float, 3> randomColor()
  {
    return {
      static_cast<float>(dist_(gen_)),
      static_cast<float>(dist_(gen_)),
      static_cast<float>(dist_(gen_))
    };
  }

  double dist(const std::pair<double,double>& a,
              const std::pair<double,double>& b)
  {
    return std::hypot(a.first - b.first, a.second - b.second);
  }

  std::vector<std::pair<double,double>> findTSP(
    std::vector<std::pair<double,double>> pts)
  {
    std::vector<std::pair<double,double>> best;
    double best_len = std::numeric_limits<double>::infinity();
    std::sort(pts.begin(), pts.end());
    do {
      double L = dist(start_pos_, pts[0]);
      for (size_t i = 0; i + 1 < pts.size(); ++i) {
        L += dist(pts[i], pts[i+1]);
      }
      L += dist(pts.back(), start_pos_);
      if (L < best_len) {
        best_len = L;
        best     = pts;
      }
    } while (std::next_permutation(pts.begin(), pts.end()));
    return best;
  }

  // ================= Members =================
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr   sub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;
  tf2_ros::Buffer                                                     tf_buffer_;
  tf2_ros::TransformListener                                           tf_listener_;

  rclcpp_action::Client<LocalizeObjAction>::SharedPtr    localize_client_;
  rclcpp_action::Server<UpdateMapAction>::SharedPtr      update_map_server_;

  std::string                                     ref_, base_frame_, map_frame_, input_topic_, output_topic_;
  double                                          threshold_{1.0}, diameter_{0.35}, height_{0.16};
  bool                                            sim_{false};

  std::vector<Ring>                               rings_;
  std::vector<std::pair<double,double>>           ring_positions_, best_path_;
  std::pair<double,double>                        start_pos_{0.0,0.0}, current_pos_{0.0,0.0};

  std::random_device                              rd_;
  std::mt19937                                    gen_;
  std::uniform_real_distribution<>                dist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
