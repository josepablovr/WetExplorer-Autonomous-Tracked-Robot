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
                                                                #include <wetexplorer_hardware/serial_device.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sensor_msgs/msg/joint_state.hpp>


using namespace std::chrono_literals;

using namespace std::chrono_literals;

enum class JointState { PAIRING, STATUS, CALIBRATE, MOVE };

class LiftingJointInterface : public rclcpp::Node {
public:
  using Calibrate = wetexplorer_hardware::action::Calibrate;
  using MoveJoint = wetexplorer_hardware::action::MoveJoint;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

  LiftingJointInterface() : Node("lifting_joint_interface"), state_(JointState::PAIRING) {
    device_name_ = this->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    is_paired_ = false;
    calibration_started_ = false;
    move_started_ = false;

    std::string stty_cmd = "sudo stty -F " + device_name_ + " 115200 min 100 time 2 -parenb -parodd -cmspar cs8 -hupcl -cstopb cread clocal -crtscts \
-ignbrk brkint ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl ixon -ixoff -iuclc -ixany -imaxbel -iutf8 \
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 \
-isig -icanon iexten -echo echoe echok -echonl -noflsh -xcase -tostop -echoprt echoctl echoke -flusho -extproc";

    int ret = system(stty_cmd.c_str());
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "stty command applied successfully");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to apply stty command");
    }

    if (serial_.connect(device_name_, baudrate_) != 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to serial device");
      rclcpp::shutdown();
      return;
    }

    // Always apply DTR/RTS after successful connection
    RCLCPP_INFO(this->get_logger(), "Applying DTR/RTS setup...");
    int modem_bits = 0;
    
    
   


    publisher_ = this->create_publisher<wetexplorer_hardware::msg::LiftingJointStatus>(
      "lifting_joint_status", 10);

    //pairing_timer_ = this->create_wall_timer(50ms, std::bind(&LiftingJointInterface::sendPairing, this));
    status_timer_ = this->create_wall_timer(10ms, std::bind(&LiftingJointInterface::loop, this));

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

  
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
  "/joint_states", 10);

  }

private:
  std::shared_ptr<GoalHandleCalibrate> active_calibrate_goal_;
  std::shared_ptr<GoalHandleMoveJoint> active_move_goal_;
  std::string device_name_;
  int baudrate_;
  bool is_paired_;
  bool calibration_started_;
  bool move_started_;
  JointState state_;
  std::string pending_move_command_;

  SerialDevice serial_;
  rclcpp::Publisher<wetexplorer_hardware::msg::LiftingJointStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr pairing_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp_action::Server<Calibrate>::SharedPtr action_server_calibrate_;
  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joint_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  void sendPairing() {
    RCLCPP_INFO(this->get_logger(), "Executing sendPairing()");
    if (is_paired_) return;

    serial_.writeLine("PAIRING\r");
    rclcpp::sleep_for(1ms);
    std::string response = serial_.readLine(1000);
    RCLCPP_INFO(this->get_logger(), "Pairing response: '%s'", response.c_str());

    if (response == "PAIRED\n") {
      RCLCPP_INFO(this->get_logger(), "Device Paired");
      is_paired_ = true;
      state_ = JointState::STATUS;
      //pairing_timer_->cancel();
    }
  }

  void loop() {
    std::string command;
    if (!is_paired_ && state_ != JointState::PAIRING) {
      state_ = JointState::PAIRING;
    }

    std::string resp;

    switch (state_) {
      case JointState::PAIRING:{
        RCLCPP_INFO(this->get_logger(), "Pairing CONTROLLER");        
        serial_.writeLine("PAIRING\r");        
        std::string response = serial_.readLine(1000);
        rclcpp::sleep_for(1ms);
        RCLCPP_INFO(this->get_logger(), "Pairing response: '%s'", response.c_str());    
        if (response == "PAIRED") {
          RCLCPP_INFO(this->get_logger(), "Device Paired");
          is_paired_ = true;
          state_ = JointState::STATUS;}        
               
        break;}
      case JointState::STATUS:
        RCLCPP_INFO(this->get_logger(), "CHECKING STATUS");  
        serial_.writeLine("STATUS\r");
        //rclcpp::sleep_for(1ms);
        resp = serial_.readLine(1000);
        RCLCPP_INFO(this->get_logger(), "Status response: '%s'", resp.c_str());    
        if (!resp.empty()) publishStatus(resp);
        break;
      case JointState::CALIBRATE:          
        rclcpp::sleep_for(1ms);
        RCLCPP_INFO(this->get_logger(), "CALIBRATING LINEAR DISTANCE");
       
        if(!calibration_started_){
          serial_.writeLine("CALIBRATE\r");
          calibration_started_ = true;}
        resp = serial_.readLine(1000);
        RCLCPP_INFO(this->get_logger(), "Calibration response: '%s'", resp.c_str());
        if (resp == "CALIBRATION DONE") {
          if (active_calibrate_goal_) {
            auto result = std::make_shared<Calibrate::Result>();
            result->success = true;
            active_calibrate_goal_->succeed(result);
            active_calibrate_goal_.reset();
          }
          calibration_started_ = false;
          state_ = JointState::STATUS;
        } else if (resp == "CALIBRATING") {
          RCLCPP_INFO(this->get_logger(), "Still calibrating...");
        }
        break;
      case JointState::MOVE:
        RCLCPP_INFO(this->get_logger(), "MOVING LIFT");
        
        if(!move_started_){
       
          RCLCPP_INFO(this->get_logger(), "Command: '%s'", pending_move_command_.c_str());
          serial_.writeLine(pending_move_command_);
          move_started_ = true;}        
        resp = serial_.readLine(1000);
        RCLCPP_INFO(this->get_logger(), "Status response: '%s'", resp.c_str());
        if (resp.find("GOAL REACHED") != std::string::npos) {
          if (active_move_goal_) {
            auto result = std::make_shared<MoveJoint::Result>();
            result->success = true;
            active_move_goal_->succeed(result);
            active_move_goal_.reset();
            move_started_ = false;
          }
          
          resp = serial_.readLine(10);
          state_ = JointState::STATUS;
        } else if (!resp.empty()) {
          publishStatus(resp);
          if (active_move_goal_) {
            auto feedback = std::make_shared<MoveJoint::Feedback>();
            feedback->distance_mm = std::stoi(resp.substr(1, 3));
            active_move_goal_->publish_feedback(feedback);
          }
        }
        break;
    }
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
    msg.calibration_state = (state_ == JointState::CALIBRATE);
    publisher_->publish(msg);
    publishJointState(msg.distance_mm);
  }


  void publishJointState(double position_mm) {
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = this->get_clock()->now();
  joint_msg.name.push_back("lift_joint");
  joint_msg.position.push_back(position_mm / 1000.0);  // convert mm to meters
  joint_state_pub_->publish(joint_msg);
}


  rclcpp_action::GoalResponse handle_goal_calibrate(const rclcpp_action::GoalUUID &, std::shared_ptr<const Calibrate::Goal>) {
    return is_paired_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse handle_goal_move_joint(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoint::Goal> goal)
  {

    RCLCPP_INFO(this->get_logger(), "Received new goal: %d mm", goal->distance_mm);
    if (active_move_goal_ && active_move_goal_->is_active()) {
      RCLCPP_INFO(this->get_logger(), "Canceling active goal to accept a new one.");
      active_move_goal_->abort(std::make_shared<MoveJoint::Result>());
      active_move_goal_.reset();
      state_ = JointState::STATUS;
      move_started_ = false;
      serial_.readLine(10);
    }
    
    return is_paired_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                      : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<void>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_calibrate(const std::shared_ptr<GoalHandleCalibrate> goal_handle) {
    active_calibrate_goal_ = goal_handle;
    state_ = JointState::CALIBRATE;
    serial_.readLine(10);
  }

  void execute_move_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
    active_move_goal_ = goal_handle;  
    std::stringstream cmd;
    cmd << "MV " << goal_handle->get_goal()->distance_mm << "\r";
    pending_move_command_ = cmd.str(); 
    state_ = JointState::MOVE;
    serial_.readLine(10);  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiftingJointInterface>());
  rclcpp::shutdown();
  return 0;
}
