// File: wetexplorer_hardware/src/lifting_joint_interface.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <wetexplorer_hardware/msg/lifting_joint_status.hpp>
#include <wetexplorer_hardware/action/calibrate.hpp>
#include <wetexplorer_hardware/action/move_joint.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

class LiftingJointInterface : public rclcpp::Node {
public:
  using Calibrate = wetexplorer_hardware::action::Calibrate;
  using MoveJoint = wetexplorer_hardware::action::MoveJoint;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

  LiftingJointInterface() : Node("lifting_joint_interface") {
    device_name_ = this->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    is_paired_ = false;
    is_executing_action_ = false;

    openSerialPort();

    publisher_ = this->create_publisher<wetexplorer_hardware::msg::LiftingJointStatus>(
      "lifting_joint_status", 10);

    pairing_timer_ = this->create_wall_timer(50ms, std::bind(&LiftingJointInterface::sendPairing, this));

    action_server_calibrate_ = rclcpp_action::create_server<Calibrate>(
      this, "calibrate",
      std::bind(&LiftingJointInterface::handle_goal_calibrate, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LiftingJointInterface::handle_cancel, this, std::placeholders::_1),
      std::bind(&LiftingJointInterface::execute_calibrate, this, std::placeholders::_1));

    action_server_move_joint_ = rclcpp_action::create_server<MoveJoint>(
      this, "move_joint",
      std::bind(&LiftingJointInterface::handle_goal_move_joint, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LiftingJointInterface::handle_cancel, this, std::placeholders::_1),
      std::bind(&LiftingJointInterface::execute_move_joint, this, std::placeholders::_1));
  }

private:
  std::string device_name_;
  int baudrate_;
  int serial_fd_;
  bool is_paired_;
  bool is_executing_action_;

  rclcpp::Publisher<wetexplorer_hardware::msg::LiftingJointStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr pairing_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp_action::Server<Calibrate>::SharedPtr action_server_calibrate_;
  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joint_;

  void openSerialPort() {
    serial_fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
      return;
    }
    struct termios tty;
    tcgetattr(serial_fd_, &tty);
    cfsetispeed(&tty, baudrate_);
    cfsetospeed(&tty, baudrate_);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  void sendPairing() {
    RCLCPP_INFO(this->get_logger(), "Executing sendPairing()");
    if (is_paired_) return;   

    ssize_t bytes_written = write(serial_fd_, "PAIRING\r", 8);
    if (bytes_written != 8) {
      RCLCPP_WARN(this->get_logger(), "PAIRING command may not have been fully written (wrote %ld bytes)", bytes_written);
    } else {
      RCLCPP_INFO(this->get_logger(), "PAIRING command written successfully (%ld bytes)", bytes_written);
    }
    std::string response = readSerial(1000);
    RCLCPP_INFO(this->get_logger(), "Pairing response: '%s'", response.c_str());
    if (response == "PAIRED") {
      RCLCPP_INFO(this->get_logger(), "Device Paired");
      is_paired_ = true;
      pairing_timer_->cancel();
      startStatusPolling();
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Device not Paired, trying again");
    }
  }

  void startStatusPolling() {
    RCLCPP_INFO(this->get_logger(), "Executing startStatusPolling()");
    status_timer_ = this->create_wall_timer(33ms, [this]() {
      if (!is_executing_action_) {
        write(serial_fd_, "STATUS\r", 7);
        std::string resp = readSerial(30);
        RCLCPP_INFO(this->get_logger(), "Pairing response: '%s'", resp.c_str());
        if (!resp.empty()) publishStatus(resp);
      }
    });
  }

  std::string readSerial(int timeout_ms) {
    std::string result;
    char c;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
      int n = read(serial_fd_, &c, 1);
      if (n > 0) {
        if (c == '\n') break;
        result += c;
      }
    }
    return result;
  }

  void publishStatus(const std::string &data) {
    if (data.length() < 13) return;
    auto msg = wetexplorer_hardware::msg::LiftingJointStatus();
    msg.stamp = this->get_clock()->now();
    msg.distance_mm = std::stoi(data.substr(1, 3));
    msg.upper_limit_switch = data[5] == '1';
    msg.lower_limit_switch = data[7] == '1';
    msg.contact_switch_1 = data[9] == '1';
    msg.contact_switch_2 = data[11] == '1';
    msg.contact_switch_3 = data[13] == '1';
    msg.calibration_state = false;  // default unless changed
    publisher_->publish(msg);
  }

  rclcpp_action::GoalResponse handle_goal_calibrate(const rclcpp_action::GoalUUID &, std::shared_ptr<const Calibrate::Goal>) {
    return is_paired_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse handle_goal_move_joint(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>) {
    return is_paired_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<void>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_calibrate(const std::shared_ptr<GoalHandleCalibrate> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing execute_calibrate()");
    is_executing_action_ = true;
    write(serial_fd_, "CALIBRATE\r", 6);
    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
      write(serial_fd_, "CALIBRATE\r", 6);
      std::string resp = readSerial(20);
      RCLCPP_INFO(this->get_logger(), "Pairing response: '%s'", resp.c_str());
      if (resp == "CALIBRATION DONE") break;
      rate.sleep();
    }
    auto result = std::make_shared<Calibrate::Result>();
    result->success = true;
    goal_handle->succeed(result);
    is_executing_action_ = false;
  }

  void execute_move_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
    is_executing_action_ = true;
    std::stringstream cmd;
    cmd << "MV " << goal_handle->get_goal()->distance_mm << "\n";
    write(serial_fd_, cmd.str().c_str(), cmd.str().length());

    auto start_time = this->now();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
      std::string resp = readSerial(10);
      if (!resp.empty()) {
        auto feedback = std::make_shared<MoveJoint::Feedback>();
        feedback->distance_mm = std::stoi(resp.substr(1, 3));
        goal_handle->publish_feedback(feedback);
        publishStatus(resp);
      }
      if ((this->now() - start_time).seconds() > 3.0) break;
      rate.sleep();
    }

    auto result = std::make_shared<MoveJoint::Result>();
    result->success = true;
    goal_handle->succeed(result);
    is_executing_action_ = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiftingJointInterface>());
  rclcpp::shutdown();
  return 0;
}
