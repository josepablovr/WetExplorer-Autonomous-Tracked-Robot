// src/pose_caller_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "wetexplorer_navigation/action/localize_object.hpp"  // the action definition

using std::placeholders::_1;
using std::placeholders::_2;
using LocalizeObj = wetexplorer_navigation::action::LocalizeObject;
using LocalizeObjGoalHandle = rclcpp_action::ClientGoalHandle<LocalizeObj>;

#include <chrono>              // add this
using namespace std::chrono_literals;   // add this (anywhere after the headers)


class PoseCaller : public rclcpp::Node
{
public:
  PoseCaller()
  : Node("pose_caller"), prev_button_state_(0)
  {
    // 1) Create subscription to /joy_teleop/joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_teleop/joy", 10,
      std::bind(&PoseCaller::joy_callback, this, _1));

    // 2) Create an action‐client for /localize_object
    localize_client_ = rclcpp_action::create_client<LocalizeObj>(
      this, "localize_object_light");

    // Wait up to a few seconds for the action server to appear
    RCLCPP_INFO(get_logger(), "Waiting for /localize_object action server...");
    if (!localize_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(),
        "/localize_object action server not available after 5 seconds");
    } else {
      RCLCPP_INFO(get_logger(),
        "/localize_object action server is now available");
    }
  }

private:
  // Called whenever a Joy message is received
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    int current_button_state = msg->buttons[0];

    // Detect rising edge of button 0
    if (current_button_state == 1 && prev_button_state_ == 0) {
      RCLCPP_INFO(get_logger(), "Button 0 pressed. Sending LocalizeObject goal...");
      send_localize_goal();
    }
    prev_button_state_ = current_button_state;
  }

  // Send an empty goal to /localize_object
  void send_localize_goal()
  {
    if (!localize_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/localize_object action server not ready, cannot send goal");
      return;
    }

    // Construct an empty goal (LocalizeObject has no fields)
    auto goal_msg = LocalizeObj::Goal();

    // Set up callbacks
    auto send_options = rclcpp_action::Client<LocalizeObj>::SendGoalOptions{};
    send_options.goal_response_callback =
      [this](std::shared_ptr<LocalizeObjGoalHandle> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "LocalizeObject goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "LocalizeObject goal accepted; waiting for result");
        }
      };

    send_options.feedback_callback =
  [this](LocalizeObjGoalHandle::SharedPtr,
         const std::shared_ptr<const LocalizeObj::Feedback> /*feedback*/) {
    RCLCPP_DEBUG(this->get_logger(), "LocalizeObject feedback received");
  };

    send_options.result_callback =
      [this](
        const LocalizeObjGoalHandle::WrappedResult & wrapped_result)
      {
        switch (wrapped_result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
            auto result = wrapped_result.result;
            auto pose_stamped = result->pose;
            RCLCPP_INFO(get_logger(),
              "LocalizeObject succeeded: pose = (%.3f, %.3f, %.3f) orientation = (%.3f, %.3f, %.3f, %.3f) in frame '%s'",
              pose_stamped.pose.position.x,
              pose_stamped.pose.position.y,
              pose_stamped.pose.position.z,
              pose_stamped.pose.orientation.x,
              pose_stamped.pose.orientation.y,
              pose_stamped.pose.orientation.z,
              pose_stamped.pose.orientation.w,
              pose_stamped.header.frame_id.c_str());
            break;
          }
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "LocalizeObject was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "LocalizeObject was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code for LocalizeObject");
            break;
        }
      };

    // Actually send the goal
    localize_client_->async_send_goal(goal_msg, send_options);
  }

  // -- members --
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp_action::Client<LocalizeObj>::SharedPtr                localize_client_;
  int prev_button_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseCaller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
