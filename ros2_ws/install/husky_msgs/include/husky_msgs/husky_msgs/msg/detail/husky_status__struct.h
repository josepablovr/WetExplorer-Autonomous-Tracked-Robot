// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice

#ifndef HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_H_
#define HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/HuskyStatus in the package husky_msgs.
typedef struct husky_msgs__msg__HuskyStatus
{
  std_msgs__msg__Header header;
  /// MCU Uptime, in ms
  uint32_t uptime;
  /// ROS Control loop frequency (PC-side)
  double ros_control_loop_freq;
  /// Current draw of platform components, in amps
  double mcu_and_user_port_current;
  double left_driver_current;
  double right_driver_current;
  /// Voltage of platform components, in volts
  double battery_voltage;
  double left_driver_voltage;
  double right_driver_voltage;
  /// Component temperatures, in C
  double left_driver_temp;
  double right_driver_temp;
  double left_motor_temp;
  double right_motor_temp;
  /// Battery capacity (Wh) and charge (%) estimate
  uint16_t capacity_estimate;
  double charge_estimate;
  /// Husky error/stop conditions
  bool timeout;
  bool lockout;
  bool e_stop;
  bool ros_pause;
  bool no_battery;
  bool current_limit;
} husky_msgs__msg__HuskyStatus;

// Struct for a sequence of husky_msgs__msg__HuskyStatus.
typedef struct husky_msgs__msg__HuskyStatus__Sequence
{
  husky_msgs__msg__HuskyStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husky_msgs__msg__HuskyStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__STRUCT_H_
