// src/ring_path_planner.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <chrono>

using GetState   = lifecycle_msgs::srv::GetState;
using Navigate   = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

class RingPathPlanner : public rclcpp::Node
{
public:
  RingPathPlanner()
  : Node("ring_path_planner"),
    got_odom_(false),
    processed_(false),
    current_goal_idx_(0)
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/global", 10,
      std::bind(&RingPathPlanner::odom_callback, this, std::placeholders::_1));

    marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_rings", 10,
      std::bind(&RingPathPlanner::marker_callback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<Navigate>(
      this, "/navigate_to_pose");

    bt_client_ = this->create_client<GetState>(
      "/bt_navigator/get_state");

    current_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/current_goal", 10);
  }

private:
  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp_action::Client<Navigate>::SharedPtr                        action_client_;
  rclcpp::Client<GetState>::SharedPtr                               bt_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr     current_goal_pub_;
  rclcpp::TimerBase::SharedPtr                                      bt_timer_;

  // Internal state
  bool got_odom_, processed_;
  std::pair<double,double> start_position_;
  std::pair<double,double> current_position_;
  std::vector<std::pair<double,double>> ring_positions_;
  std::vector<std::pair<double,double>> best_path_;
  size_t current_goal_idx_;

  // latch and update current_position_
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_.first  = msg->pose.pose.position.x;
    current_position_.second = msg->pose.pose.position.y;
    if (!got_odom_) {
      start_position_ = current_position_;
      got_odom_ = true;
      RCLCPP_INFO(get_logger(),
        "Start position set to (%.2f, %.2f)",
        start_position_.first, start_position_.second);
    }
  }

  // collect ring markers, compute tour, kick off BT polling
  void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (!got_odom_ || processed_) return;
    for (auto &m : msg->markers) {
      ring_positions_.emplace_back(m.pose.position.x, m.pose.position.y);
    }
    if (ring_positions_.empty()) {
      RCLCPP_WARN(get_logger(), "No rings on /map_rings");
      return;
    }
    processed_ = true;
    best_path_ = find_shortest_path();
    current_goal_idx_ = 0;
    bt_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RingPathPlanner::check_bt_timer, this));
  }

  // poll get_state service
  void check_bt_timer()
  {
    if (!bt_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/bt_navigator/get_state not ready, retrying...");
      return;
    }
    bt_timer_->cancel();
    auto req = std::make_shared<GetState::Request>();
    bt_client_->async_send_request(
      req,
      std::bind(&RingPathPlanner::on_bt_response, this, std::placeholders::_1));
  }

  // on BT response, start navigation
  void on_bt_response(rclcpp::Client<GetState>::SharedFuture future)
  {
    auto resp = future.get();
    if (resp->current_state.id == 3) {
      RCLCPP_INFO(get_logger(), "BT ACTIVE, sending first goal");
      send_next_goal();
    } else {
      RCLCPP_INFO(get_logger(),
        "BT state=%d(%s), retrying...",
        resp->current_state.id, resp->current_state.label.c_str());
      bt_timer_->reset();
    }
  }

  // Euclid distance
  double euclidean_distance(const std::pair<double,double>& a,
                            const std::pair<double,double>& b)
  {
    return std::hypot(a.first - b.first, a.second - b.second);
  }

  // brute-force TSP
  std::vector<std::pair<double,double>> find_shortest_path()
  {
    auto pts = ring_positions_;
    std::vector<std::pair<double,double>> best;
    double best_len = std::numeric_limits<double>::infinity();
    std::sort(pts.begin(), pts.end());
    do {
      double len = euclidean_distance(start_position_, pts[0]);
      for (size_t i=0; i+1<pts.size(); ++i)
        len += euclidean_distance(pts[i], pts[i+1]);
      len += euclidean_distance(pts.back(), start_position_);
      if (len < best_len) { best_len = len; best = pts; }
    } while (std::next_permutation(pts.begin(), pts.end()));
    RCLCPP_INFO(get_logger(), "Shortest tour = %.2f m", best_len);
    return best;
  }

  // send goal and publish a marker
  void send_next_goal()
  {
    bool is_return = (  >= best_path_.size());
    auto target = is_return ? start_position_ : best_path_[current_goal_idx_];
    auto current__pt = best_path_[current_goal_idx_];
    auto next_pt = is_return
      ? start_position_
      : (current_goal_idx_+1 < best_path_.size()
         ? best_path_[current_goal_idx_+1]
         : start_position_);

    double dx_h = current__pt.first  - current_position_.first;
    double dy_h = current__pt.second - current_position_.second;
    double yaw  = std::atan2(dy_h, dx_h);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose unavailable");
      return;
    }

    double gx = target.first  - std::cos(yaw);
    double gy = target.second - std::sin(yaw);

    // publish marker as before...
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "current_goal";
    marker.id = int(current_goal_idx_);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = gx;
    marker.pose.position.y = gy;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.z = std::sin(yaw/2.0);
    marker.pose.orientation.w = std::cos(yaw/2.0);
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;
    current_goal_pub_->publish(marker);

    // now build the NavigateToPose goal Msg correctly:
    Navigate::Goal goal_msg;
    goal_msg.pose.header = marker.header;  // stamp + frame
    goal_msg.pose.pose   = marker.pose;    // position + orientation

    rclcpp_action::Client<Navigate>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this,is_return](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal rejected");
        }
      };
    opts.result_callback =
      [this,is_return](const GoalHandle::WrappedResult & res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (!is_return) {
            RCLCPP_INFO(get_logger(), "Reached ring %zu", current_goal_idx_+1);
            current_goal_idx_++;
            send_next_goal();
          } else {
            RCLCPP_INFO(get_logger(), "Returned to start — mission complete");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Navigation failed (code %d)",
            static_cast<int>(res.code));
        }
      };

    action_client_->async_send_goal(goal_msg, opts);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingPathPlanner>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
