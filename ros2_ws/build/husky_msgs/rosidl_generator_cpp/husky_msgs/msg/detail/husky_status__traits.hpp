// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice

#ifndef HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__TRAITS_HPP_
#define HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husky_msgs/msg/detail/husky_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace husky_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HuskyStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: uptime
  {
    out << "uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.uptime, out);
    out << ", ";
  }

  // member: ros_control_loop_freq
  {
    out << "ros_control_loop_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.ros_control_loop_freq, out);
    out << ", ";
  }

  // member: mcu_and_user_port_current
  {
    out << "mcu_and_user_port_current: ";
    rosidl_generator_traits::value_to_yaml(msg.mcu_and_user_port_current, out);
    out << ", ";
  }

  // member: left_driver_current
  {
    out << "left_driver_current: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_current, out);
    out << ", ";
  }

  // member: right_driver_current
  {
    out << "right_driver_current: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_current, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: left_driver_voltage
  {
    out << "left_driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_voltage, out);
    out << ", ";
  }

  // member: right_driver_voltage
  {
    out << "right_driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_voltage, out);
    out << ", ";
  }

  // member: left_driver_temp
  {
    out << "left_driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_temp, out);
    out << ", ";
  }

  // member: right_driver_temp
  {
    out << "right_driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_temp, out);
    out << ", ";
  }

  // member: left_motor_temp
  {
    out << "left_motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor_temp, out);
    out << ", ";
  }

  // member: right_motor_temp
  {
    out << "right_motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor_temp, out);
    out << ", ";
  }

  // member: capacity_estimate
  {
    out << "capacity_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.capacity_estimate, out);
    out << ", ";
  }

  // member: charge_estimate
  {
    out << "charge_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.charge_estimate, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: lockout
  {
    out << "lockout: ";
    rosidl_generator_traits::value_to_yaml(msg.lockout, out);
    out << ", ";
  }

  // member: e_stop
  {
    out << "e_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_stop, out);
    out << ", ";
  }

  // member: ros_pause
  {
    out << "ros_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.ros_pause, out);
    out << ", ";
  }

  // member: no_battery
  {
    out << "no_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.no_battery, out);
    out << ", ";
  }

  // member: current_limit
  {
    out << "current_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.current_limit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HuskyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: uptime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.uptime, out);
    out << "\n";
  }

  // member: ros_control_loop_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ros_control_loop_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.ros_control_loop_freq, out);
    out << "\n";
  }

  // member: mcu_and_user_port_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mcu_and_user_port_current: ";
    rosidl_generator_traits::value_to_yaml(msg.mcu_and_user_port_current, out);
    out << "\n";
  }

  // member: left_driver_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_driver_current: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_current, out);
    out << "\n";
  }

  // member: right_driver_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_driver_current: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_current, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: left_driver_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_voltage, out);
    out << "\n";
  }

  // member: right_driver_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_voltage, out);
    out << "\n";
  }

  // member: left_driver_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_driver_temp, out);
    out << "\n";
  }

  // member: right_driver_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_driver_temp, out);
    out << "\n";
  }

  // member: left_motor_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor_temp, out);
    out << "\n";
  }

  // member: right_motor_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor_temp, out);
    out << "\n";
  }

  // member: capacity_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "capacity_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.capacity_estimate, out);
    out << "\n";
  }

  // member: charge_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charge_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.charge_estimate, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

  // member: lockout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lockout: ";
    rosidl_generator_traits::value_to_yaml(msg.lockout, out);
    out << "\n";
  }

  // member: e_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_stop, out);
    out << "\n";
  }

  // member: ros_pause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ros_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.ros_pause, out);
    out << "\n";
  }

  // member: no_battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "no_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.no_battery, out);
    out << "\n";
  }

  // member: current_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.current_limit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HuskyStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace husky_msgs

namespace rosidl_generator_traits
{

[[deprecated("use husky_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const husky_msgs::msg::HuskyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  husky_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husky_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husky_msgs::msg::HuskyStatus & msg)
{
  return husky_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husky_msgs::msg::HuskyStatus>()
{
  return "husky_msgs::msg::HuskyStatus";
}

template<>
inline const char * name<husky_msgs::msg::HuskyStatus>()
{
  return "husky_msgs/msg/HuskyStatus";
}

template<>
struct has_fixed_size<husky_msgs::msg::HuskyStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<husky_msgs::msg::HuskyStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<husky_msgs::msg::HuskyStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__TRAITS_HPP_
