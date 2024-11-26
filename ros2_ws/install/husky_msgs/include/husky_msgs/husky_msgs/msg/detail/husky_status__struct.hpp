// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice

#ifndef HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_HPP_
#define HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__husky_msgs__msg__HuskyStatus __attribute__((deprecated))
#else
# define DEPRECATED__husky_msgs__msg__HuskyStatus __declspec(deprecated)
#endif

namespace husky_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HuskyStatus_
{
  using Type = HuskyStatus_<ContainerAllocator>;

  explicit HuskyStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uptime = 0ul;
      this->ros_control_loop_freq = 0.0;
      this->mcu_and_user_port_current = 0.0;
      this->left_driver_current = 0.0;
      this->right_driver_current = 0.0;
      this->battery_voltage = 0.0;
      this->left_driver_voltage = 0.0;
      this->right_driver_voltage = 0.0;
      this->left_driver_temp = 0.0;
      this->right_driver_temp = 0.0;
      this->left_motor_temp = 0.0;
      this->right_motor_temp = 0.0;
      this->capacity_estimate = 0;
      this->charge_estimate = 0.0;
      this->timeout = false;
      this->lockout = false;
      this->e_stop = false;
      this->ros_pause = false;
      this->no_battery = false;
      this->current_limit = false;
    }
  }

  explicit HuskyStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uptime = 0ul;
      this->ros_control_loop_freq = 0.0;
      this->mcu_and_user_port_current = 0.0;
      this->left_driver_current = 0.0;
      this->right_driver_current = 0.0;
      this->battery_voltage = 0.0;
      this->left_driver_voltage = 0.0;
      this->right_driver_voltage = 0.0;
      this->left_driver_temp = 0.0;
      this->right_driver_temp = 0.0;
      this->left_motor_temp = 0.0;
      this->right_motor_temp = 0.0;
      this->capacity_estimate = 0;
      this->charge_estimate = 0.0;
      this->timeout = false;
      this->lockout = false;
      this->e_stop = false;
      this->ros_pause = false;
      this->no_battery = false;
      this->current_limit = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _uptime_type =
    uint32_t;
  _uptime_type uptime;
  using _ros_control_loop_freq_type =
    double;
  _ros_control_loop_freq_type ros_control_loop_freq;
  using _mcu_and_user_port_current_type =
    double;
  _mcu_and_user_port_current_type mcu_and_user_port_current;
  using _left_driver_current_type =
    double;
  _left_driver_current_type left_driver_current;
  using _right_driver_current_type =
    double;
  _right_driver_current_type right_driver_current;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _left_driver_voltage_type =
    double;
  _left_driver_voltage_type left_driver_voltage;
  using _right_driver_voltage_type =
    double;
  _right_driver_voltage_type right_driver_voltage;
  using _left_driver_temp_type =
    double;
  _left_driver_temp_type left_driver_temp;
  using _right_driver_temp_type =
    double;
  _right_driver_temp_type right_driver_temp;
  using _left_motor_temp_type =
    double;
  _left_motor_temp_type left_motor_temp;
  using _right_motor_temp_type =
    double;
  _right_motor_temp_type right_motor_temp;
  using _capacity_estimate_type =
    uint16_t;
  _capacity_estimate_type capacity_estimate;
  using _charge_estimate_type =
    double;
  _charge_estimate_type charge_estimate;
  using _timeout_type =
    bool;
  _timeout_type timeout;
  using _lockout_type =
    bool;
  _lockout_type lockout;
  using _e_stop_type =
    bool;
  _e_stop_type e_stop;
  using _ros_pause_type =
    bool;
  _ros_pause_type ros_pause;
  using _no_battery_type =
    bool;
  _no_battery_type no_battery;
  using _current_limit_type =
    bool;
  _current_limit_type current_limit;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__uptime(
    const uint32_t & _arg)
  {
    this->uptime = _arg;
    return *this;
  }
  Type & set__ros_control_loop_freq(
    const double & _arg)
  {
    this->ros_control_loop_freq = _arg;
    return *this;
  }
  Type & set__mcu_and_user_port_current(
    const double & _arg)
  {
    this->mcu_and_user_port_current = _arg;
    return *this;
  }
  Type & set__left_driver_current(
    const double & _arg)
  {
    this->left_driver_current = _arg;
    return *this;
  }
  Type & set__right_driver_current(
    const double & _arg)
  {
    this->right_driver_current = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__left_driver_voltage(
    const double & _arg)
  {
    this->left_driver_voltage = _arg;
    return *this;
  }
  Type & set__right_driver_voltage(
    const double & _arg)
  {
    this->right_driver_voltage = _arg;
    return *this;
  }
  Type & set__left_driver_temp(
    const double & _arg)
  {
    this->left_driver_temp = _arg;
    return *this;
  }
  Type & set__right_driver_temp(
    const double & _arg)
  {
    this->right_driver_temp = _arg;
    return *this;
  }
  Type & set__left_motor_temp(
    const double & _arg)
  {
    this->left_motor_temp = _arg;
    return *this;
  }
  Type & set__right_motor_temp(
    const double & _arg)
  {
    this->right_motor_temp = _arg;
    return *this;
  }
  Type & set__capacity_estimate(
    const uint16_t & _arg)
  {
    this->capacity_estimate = _arg;
    return *this;
  }
  Type & set__charge_estimate(
    const double & _arg)
  {
    this->charge_estimate = _arg;
    return *this;
  }
  Type & set__timeout(
    const bool & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__lockout(
    const bool & _arg)
  {
    this->lockout = _arg;
    return *this;
  }
  Type & set__e_stop(
    const bool & _arg)
  {
    this->e_stop = _arg;
    return *this;
  }
  Type & set__ros_pause(
    const bool & _arg)
  {
    this->ros_pause = _arg;
    return *this;
  }
  Type & set__no_battery(
    const bool & _arg)
  {
    this->no_battery = _arg;
    return *this;
  }
  Type & set__current_limit(
    const bool & _arg)
  {
    this->current_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husky_msgs::msg::HuskyStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const husky_msgs::msg::HuskyStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husky_msgs::msg::HuskyStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husky_msgs::msg::HuskyStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husky_msgs__msg__HuskyStatus
    std::shared_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husky_msgs__msg__HuskyStatus
    std::shared_ptr<husky_msgs::msg::HuskyStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HuskyStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->uptime != other.uptime) {
      return false;
    }
    if (this->ros_control_loop_freq != other.ros_control_loop_freq) {
      return false;
    }
    if (this->mcu_and_user_port_current != other.mcu_and_user_port_current) {
      return false;
    }
    if (this->left_driver_current != other.left_driver_current) {
      return false;
    }
    if (this->right_driver_current != other.right_driver_current) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->left_driver_voltage != other.left_driver_voltage) {
      return false;
    }
    if (this->right_driver_voltage != other.right_driver_voltage) {
      return false;
    }
    if (this->left_driver_temp != other.left_driver_temp) {
      return false;
    }
    if (this->right_driver_temp != other.right_driver_temp) {
      return false;
    }
    if (this->left_motor_temp != other.left_motor_temp) {
      return false;
    }
    if (this->right_motor_temp != other.right_motor_temp) {
      return false;
    }
    if (this->capacity_estimate != other.capacity_estimate) {
      return false;
    }
    if (this->charge_estimate != other.charge_estimate) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->lockout != other.lockout) {
      return false;
    }
    if (this->e_stop != other.e_stop) {
      return false;
    }
    if (this->ros_pause != other.ros_pause) {
      return false;
    }
    if (this->no_battery != other.no_battery) {
      return false;
    }
    if (this->current_limit != other.current_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const HuskyStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HuskyStatus_

// alias to use template instance with default allocator
using HuskyStatus =
  husky_msgs::msg::HuskyStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husky_msgs

#endif  // HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_HPP_
