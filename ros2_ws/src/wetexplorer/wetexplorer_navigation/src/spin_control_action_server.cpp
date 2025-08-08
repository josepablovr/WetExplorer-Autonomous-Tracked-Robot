// src/spin_control_action_server.cpp
//
// A simple PI yaw controller exposed as an action server at the topic
// "/spin_control".  The node subscribes to odometry, accepts a target yaw
// (rad) goal and drives the robot until the yaw error is within a small
// tolerance.
//
// © 2025 – BSD‑3‑Clause, see accompanying LICENSE file.

#include <memory>
#include <cmath>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wetexplorer_navigation/action/spin_yaw.hpp"   // Goal: target yaw (rad)

using SpinYaw    = wetexplorer_navigation::action::SpinYaw;
using GoalHandle = rclcpp_action::ServerGoalHandle<SpinYaw>;

class SpinControlActionServer : public rclcpp::Node
{
public:
  SpinControlActionServer()
  : Node("spin_control_action_server"),
    goal_active_(false), steady_count_(0)
  {
    // Frame reference parameter (choose odometry topic)
    this->declare_parameter<std::string>("ref", "map");
    this->get_parameter("ref", ref_frame_);

    std::string odom_topic = (ref_frame_ == "odom") ? "/odometry/local"
                                                     : "/odometry/global";

    // Publisher & subscriber
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/commands/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20,
      std::bind(&SpinControlActionServer::odomCallback, this, std::placeholders::_1));

    // Action server – exposes "/spin_control"
    action_server_ = rclcpp_action::create_server<SpinYaw>(
      this,
      "/spin_control",
      std::bind(&SpinControlActionServer::handleGoal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SpinControlActionServer::handleCancel,   this, std::placeholders::_1),
      std::bind(&SpinControlActionServer::handleAccepted, this, std::placeholders::_1));

    // Controller constants (default; can be made params later)
    tolerance_   = 0.087;   // rad ≈ 5.0°
    max_w_       = 0.2;    // rad / s
    Kp_          = 1.0;    // proportional gain
    Ki_          = 0.01;    // integral gain
    integral_    = 0.0;
  }

private:
  // — ROS interfaces —
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<SpinYaw>::SharedPtr action_server_;

  // — State —
  double cur_yaw_ {0.0};

  // Active goal
  bool   goal_active_;
  double target_yaw_;
  size_t steady_count_;

  // Controller params
  double tolerance_, max_w_;
  double Kp_, Ki_;
  double integral_;

  // Selected odometry frame
  std::string ref_frame_;

  // ========== Helpers ==========
  static double normalizeAngle(double a)
  {
    // Wrap angle to [-π, π]
    a = std::fmod(a + M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    return a - M_PI;
  }

  // ========== Odometry callback ==========
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &q_msg = msg->pose.pose.orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    cur_yaw_ = yaw;
  }

  // ========== Action callbacks ==========
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SpinYaw::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Spin goal received: target %.3f rad", goal->target_yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> /*gh*/)
  {
    RCLCPP_INFO(get_logger(), "Spin goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&SpinControlActionServer::execute, this, goal_handle)}.detach();
  }

  // ========== Execution loop ==========
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result   = std::make_shared<SpinYaw::Result>();
    auto feedback = std::make_shared<SpinYaw::Feedback>();

    target_yaw_   = goal_handle->get_goal()->target_yaw;
    integral_     = 0.0;
    goal_active_  = true;
    steady_count_ = 0;

    rclcpp::Rate rate(20);   // 25 Hz control loop

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stopRobot();
        result->success = false;
        goal_handle->canceled(result);
        goal_active_ = false;
        return;
      }

      // — Compute error —
      double error = normalizeAngle(target_yaw_ - cur_yaw_);
      feedback->remaining_angle = error;
      goal_handle->publish_feedback(feedback);

      if (std::abs(error) < tolerance_) {
        ++steady_count_;
      } else {
        steady_count_ = 0;
      }

      if (steady_count_ >= 100) {  // ~4 s within tolerance
        stopRobot();
        result->success = true;
        goal_handle->succeed(result);
        goal_active_ = false;
        RCLCPP_INFO(get_logger(), "Spin goal reached (|error| < %.3f rad)", tolerance_);
        return;
      }

      // — PI control —
      integral_ += error;
      integral_ = std::clamp(integral_, -1.0, 1.0);

      double w = Kp_ * error + Ki_ * integral_;
      w = std::clamp(w, -max_w_, max_w_);

      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = w;
      cmd.linear.x  = 0.0;
      cmd_vel_pub_->publish(cmd);

      rate.sleep();
    }

    // Abort if we drop out of the loop unexpectedly
    stopRobot();
    result->success = false;
    goal_handle->abort(result);
    goal_active_ = false;
  }

  void stopRobot()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinControlActionServer>());
  rclcpp::shutdown();
  return 0;
}
