// File: joy_lift_controller.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wetexplorer_hardware/action/move_joint.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class JoyLiftController : public rclcpp::Node {
public:
  using MoveJoint = wetexplorer_hardware::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;

  JoyLiftController() : Node("joy_lift_controller"), current_position_mm_(0) {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_teleop/joy", 1, std::bind(&JoyLiftController::joyCallback, this, _1));

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JoyLiftController::jointCallback, this, _1));

    action_client_ = rclcpp_action::create_client<MoveJoint>(this, "/move_joint");
    direction_ = 0;

    RCLCPP_INFO(this->get_logger(), "JoyLiftController node started.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp_action::Client<MoveJoint>::SharedPtr action_client_;

  double current_position_mm_;
  double direction_;

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "lift_joint") {
        current_position_mm_ = msg->position[i] * 1000.0; // convert from meters to mm
        break;
      }
    }
  }



  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.empty()) return;

  
    float direction = msg->axes[msg->axes.size() - 1];



    int new_goal = static_cast<int>(current_position_mm_);

    //RCLCPP_INFO(this->get_logger(), "Current Distance: %d mm", new_goal);
    if (direction == 1.0f) {
      new_goal -=  30;
      direction_ = -1.0f;
      new_goal = std::clamp(new_goal, 0, 300);
      sendGoal(new_goal);
    } else if (direction == -1.0f) {
      new_goal += 30;
      direction_ = 1.0f;
      new_goal = std::clamp(new_goal, 0, 300);
      sendGoal(new_goal);
    }
    else if (direction == 0.0f){
      new_goal = new_goal + direction_*5.0;
      if (direction_ != 0.0) {
        new_goal = std::clamp(new_goal, 0, 300);
        sendGoal(new_goal);
      }
      direction_ = 0.0;
    }
    
    
  }
  
  void sendGoal(int target_mm) {
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "MoveJoint action server not available.");
      return;
    }

    auto goal_msg = MoveJoint::Goal();
    goal_msg.distance_mm = target_mm;

    RCLCPP_INFO(this->get_logger(), "Sending lift_joint goal: %d mm", target_mm);

    action_client_->async_send_goal(goal_msg);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyLiftController>());
  rclcpp::shutdown();
  return 0;
}
