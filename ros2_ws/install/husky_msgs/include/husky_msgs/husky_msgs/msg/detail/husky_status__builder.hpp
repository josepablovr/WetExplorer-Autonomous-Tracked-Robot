// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice

#ifndef HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__BUILDER_HPP_
#define HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husky_msgs/msg/detail/husky_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husky_msgs
{

namespace msg
{

namespace builder
{

class Init_HuskyStatus_current_limit
{
public:
  explicit Init_HuskyStatus_current_limit(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  ::husky_msgs::msg::HuskyStatus current_limit(::husky_msgs::msg::HuskyStatus::_current_limit_type arg)
  {
    msg_.current_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_no_battery
{
public:
  explicit Init_HuskyStatus_no_battery(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_current_limit no_battery(::husky_msgs::msg::HuskyStatus::_no_battery_type arg)
  {
    msg_.no_battery = std::move(arg);
    return Init_HuskyStatus_current_limit(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_ros_pause
{
public:
  explicit Init_HuskyStatus_ros_pause(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_no_battery ros_pause(::husky_msgs::msg::HuskyStatus::_ros_pause_type arg)
  {
    msg_.ros_pause = std::move(arg);
    return Init_HuskyStatus_no_battery(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_e_stop
{
public:
  explicit Init_HuskyStatus_e_stop(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_ros_pause e_stop(::husky_msgs::msg::HuskyStatus::_e_stop_type arg)
  {
    msg_.e_stop = std::move(arg);
    return Init_HuskyStatus_ros_pause(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_lockout
{
public:
  explicit Init_HuskyStatus_lockout(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_e_stop lockout(::husky_msgs::msg::HuskyStatus::_lockout_type arg)
  {
    msg_.lockout = std::move(arg);
    return Init_HuskyStatus_e_stop(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_timeout
{
public:
  explicit Init_HuskyStatus_timeout(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_lockout timeout(::husky_msgs::msg::HuskyStatus::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_HuskyStatus_lockout(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_charge_estimate
{
public:
  explicit Init_HuskyStatus_charge_estimate(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_timeout charge_estimate(::husky_msgs::msg::HuskyStatus::_charge_estimate_type arg)
  {
    msg_.charge_estimate = std::move(arg);
    return Init_HuskyStatus_timeout(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_capacity_estimate
{
public:
  explicit Init_HuskyStatus_capacity_estimate(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_charge_estimate capacity_estimate(::husky_msgs::msg::HuskyStatus::_capacity_estimate_type arg)
  {
    msg_.capacity_estimate = std::move(arg);
    return Init_HuskyStatus_charge_estimate(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_right_motor_temp
{
public:
  explicit Init_HuskyStatus_right_motor_temp(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_capacity_estimate right_motor_temp(::husky_msgs::msg::HuskyStatus::_right_motor_temp_type arg)
  {
    msg_.right_motor_temp = std::move(arg);
    return Init_HuskyStatus_capacity_estimate(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_left_motor_temp
{
public:
  explicit Init_HuskyStatus_left_motor_temp(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_right_motor_temp left_motor_temp(::husky_msgs::msg::HuskyStatus::_left_motor_temp_type arg)
  {
    msg_.left_motor_temp = std::move(arg);
    return Init_HuskyStatus_right_motor_temp(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_right_driver_temp
{
public:
  explicit Init_HuskyStatus_right_driver_temp(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_left_motor_temp right_driver_temp(::husky_msgs::msg::HuskyStatus::_right_driver_temp_type arg)
  {
    msg_.right_driver_temp = std::move(arg);
    return Init_HuskyStatus_left_motor_temp(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_left_driver_temp
{
public:
  explicit Init_HuskyStatus_left_driver_temp(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_right_driver_temp left_driver_temp(::husky_msgs::msg::HuskyStatus::_left_driver_temp_type arg)
  {
    msg_.left_driver_temp = std::move(arg);
    return Init_HuskyStatus_right_driver_temp(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_right_driver_voltage
{
public:
  explicit Init_HuskyStatus_right_driver_voltage(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_left_driver_temp right_driver_voltage(::husky_msgs::msg::HuskyStatus::_right_driver_voltage_type arg)
  {
    msg_.right_driver_voltage = std::move(arg);
    return Init_HuskyStatus_left_driver_temp(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_left_driver_voltage
{
public:
  explicit Init_HuskyStatus_left_driver_voltage(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_right_driver_voltage left_driver_voltage(::husky_msgs::msg::HuskyStatus::_left_driver_voltage_type arg)
  {
    msg_.left_driver_voltage = std::move(arg);
    return Init_HuskyStatus_right_driver_voltage(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_battery_voltage
{
public:
  explicit Init_HuskyStatus_battery_voltage(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_left_driver_voltage battery_voltage(::husky_msgs::msg::HuskyStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_HuskyStatus_left_driver_voltage(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_right_driver_current
{
public:
  explicit Init_HuskyStatus_right_driver_current(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_battery_voltage right_driver_current(::husky_msgs::msg::HuskyStatus::_right_driver_current_type arg)
  {
    msg_.right_driver_current = std::move(arg);
    return Init_HuskyStatus_battery_voltage(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_left_driver_current
{
public:
  explicit Init_HuskyStatus_left_driver_current(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_right_driver_current left_driver_current(::husky_msgs::msg::HuskyStatus::_left_driver_current_type arg)
  {
    msg_.left_driver_current = std::move(arg);
    return Init_HuskyStatus_right_driver_current(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_mcu_and_user_port_current
{
public:
  explicit Init_HuskyStatus_mcu_and_user_port_current(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_left_driver_current mcu_and_user_port_current(::husky_msgs::msg::HuskyStatus::_mcu_and_user_port_current_type arg)
  {
    msg_.mcu_and_user_port_current = std::move(arg);
    return Init_HuskyStatus_left_driver_current(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_ros_control_loop_freq
{
public:
  explicit Init_HuskyStatus_ros_control_loop_freq(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_mcu_and_user_port_current ros_control_loop_freq(::husky_msgs::msg::HuskyStatus::_ros_control_loop_freq_type arg)
  {
    msg_.ros_control_loop_freq = std::move(arg);
    return Init_HuskyStatus_mcu_and_user_port_current(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_uptime
{
public:
  explicit Init_HuskyStatus_uptime(::husky_msgs::msg::HuskyStatus & msg)
  : msg_(msg)
  {}
  Init_HuskyStatus_ros_control_loop_freq uptime(::husky_msgs::msg::HuskyStatus::_uptime_type arg)
  {
    msg_.uptime = std::move(arg);
    return Init_HuskyStatus_ros_control_loop_freq(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

class Init_HuskyStatus_header
{
public:
  Init_HuskyStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HuskyStatus_uptime header(::husky_msgs::msg::HuskyStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HuskyStatus_uptime(msg_);
  }

private:
  ::husky_msgs::msg::HuskyStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husky_msgs::msg::HuskyStatus>()
{
  return husky_msgs::msg::builder::Init_HuskyStatus_header();
}

}  // namespace husky_msgs

#endif  // HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__BUILDER_HPP_
