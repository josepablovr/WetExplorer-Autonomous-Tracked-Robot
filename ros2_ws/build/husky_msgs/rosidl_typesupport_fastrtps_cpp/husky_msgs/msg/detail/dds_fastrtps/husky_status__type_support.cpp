// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice
#include "husky_msgs/msg/detail/husky_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husky_msgs/msg/detail/husky_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace husky_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husky_msgs
cdr_serialize(
  const husky_msgs::msg::HuskyStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: uptime
  cdr << ros_message.uptime;
  // Member: ros_control_loop_freq
  cdr << ros_message.ros_control_loop_freq;
  // Member: mcu_and_user_port_current
  cdr << ros_message.mcu_and_user_port_current;
  // Member: left_driver_current
  cdr << ros_message.left_driver_current;
  // Member: right_driver_current
  cdr << ros_message.right_driver_current;
  // Member: battery_voltage
  cdr << ros_message.battery_voltage;
  // Member: left_driver_voltage
  cdr << ros_message.left_driver_voltage;
  // Member: right_driver_voltage
  cdr << ros_message.right_driver_voltage;
  // Member: left_driver_temp
  cdr << ros_message.left_driver_temp;
  // Member: right_driver_temp
  cdr << ros_message.right_driver_temp;
  // Member: left_motor_temp
  cdr << ros_message.left_motor_temp;
  // Member: right_motor_temp
  cdr << ros_message.right_motor_temp;
  // Member: capacity_estimate
  cdr << ros_message.capacity_estimate;
  // Member: charge_estimate
  cdr << ros_message.charge_estimate;
  // Member: timeout
  cdr << (ros_message.timeout ? true : false);
  // Member: lockout
  cdr << (ros_message.lockout ? true : false);
  // Member: e_stop
  cdr << (ros_message.e_stop ? true : false);
  // Member: ros_pause
  cdr << (ros_message.ros_pause ? true : false);
  // Member: no_battery
  cdr << (ros_message.no_battery ? true : false);
  // Member: current_limit
  cdr << (ros_message.current_limit ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husky_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husky_msgs::msg::HuskyStatus & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: uptime
  cdr >> ros_message.uptime;

  // Member: ros_control_loop_freq
  cdr >> ros_message.ros_control_loop_freq;

  // Member: mcu_and_user_port_current
  cdr >> ros_message.mcu_and_user_port_current;

  // Member: left_driver_current
  cdr >> ros_message.left_driver_current;

  // Member: right_driver_current
  cdr >> ros_message.right_driver_current;

  // Member: battery_voltage
  cdr >> ros_message.battery_voltage;

  // Member: left_driver_voltage
  cdr >> ros_message.left_driver_voltage;

  // Member: right_driver_voltage
  cdr >> ros_message.right_driver_voltage;

  // Member: left_driver_temp
  cdr >> ros_message.left_driver_temp;

  // Member: right_driver_temp
  cdr >> ros_message.right_driver_temp;

  // Member: left_motor_temp
  cdr >> ros_message.left_motor_temp;

  // Member: right_motor_temp
  cdr >> ros_message.right_motor_temp;

  // Member: capacity_estimate
  cdr >> ros_message.capacity_estimate;

  // Member: charge_estimate
  cdr >> ros_message.charge_estimate;

  // Member: timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.timeout = tmp ? true : false;
  }

  // Member: lockout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.lockout = tmp ? true : false;
  }

  // Member: e_stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.e_stop = tmp ? true : false;
  }

  // Member: ros_pause
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ros_pause = tmp ? true : false;
  }

  // Member: no_battery
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.no_battery = tmp ? true : false;
  }

  // Member: current_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.current_limit = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husky_msgs
get_serialized_size(
  const husky_msgs::msg::HuskyStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: uptime
  {
    size_t item_size = sizeof(ros_message.uptime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ros_control_loop_freq
  {
    size_t item_size = sizeof(ros_message.ros_control_loop_freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mcu_and_user_port_current
  {
    size_t item_size = sizeof(ros_message.mcu_and_user_port_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_driver_current
  {
    size_t item_size = sizeof(ros_message.left_driver_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_driver_current
  {
    size_t item_size = sizeof(ros_message.right_driver_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_driver_voltage
  {
    size_t item_size = sizeof(ros_message.left_driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_driver_voltage
  {
    size_t item_size = sizeof(ros_message.right_driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_driver_temp
  {
    size_t item_size = sizeof(ros_message.left_driver_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_driver_temp
  {
    size_t item_size = sizeof(ros_message.right_driver_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_motor_temp
  {
    size_t item_size = sizeof(ros_message.left_motor_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_motor_temp
  {
    size_t item_size = sizeof(ros_message.right_motor_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: capacity_estimate
  {
    size_t item_size = sizeof(ros_message.capacity_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: charge_estimate
  {
    size_t item_size = sizeof(ros_message.charge_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timeout
  {
    size_t item_size = sizeof(ros_message.timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lockout
  {
    size_t item_size = sizeof(ros_message.lockout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: e_stop
  {
    size_t item_size = sizeof(ros_message.e_stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ros_pause
  {
    size_t item_size = sizeof(ros_message.ros_pause);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: no_battery
  {
    size_t item_size = sizeof(ros_message.no_battery);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_limit
  {
    size_t item_size = sizeof(ros_message.current_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husky_msgs
max_serialized_size_HuskyStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: uptime
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ros_control_loop_freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: mcu_and_user_port_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_driver_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_driver_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_driver_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_driver_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_driver_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_driver_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_motor_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_motor_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: capacity_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: charge_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: timeout
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: lockout
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: e_stop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ros_pause
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: no_battery
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = husky_msgs::msg::HuskyStatus;
    is_plain =
      (
      offsetof(DataType, current_limit) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _HuskyStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const husky_msgs::msg::HuskyStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HuskyStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husky_msgs::msg::HuskyStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HuskyStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husky_msgs::msg::HuskyStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HuskyStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_HuskyStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _HuskyStatus__callbacks = {
  "husky_msgs::msg",
  "HuskyStatus",
  _HuskyStatus__cdr_serialize,
  _HuskyStatus__cdr_deserialize,
  _HuskyStatus__get_serialized_size,
  _HuskyStatus__max_serialized_size
};

static rosidl_message_type_support_t _HuskyStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HuskyStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husky_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husky_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husky_msgs::msg::HuskyStatus>()
{
  return &husky_msgs::msg::typesupport_fastrtps_cpp::_HuskyStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husky_msgs, msg, HuskyStatus)() {
  return &husky_msgs::msg::typesupport_fastrtps_cpp::_HuskyStatus__handle;
}

#ifdef __cplusplus
}
#endif
