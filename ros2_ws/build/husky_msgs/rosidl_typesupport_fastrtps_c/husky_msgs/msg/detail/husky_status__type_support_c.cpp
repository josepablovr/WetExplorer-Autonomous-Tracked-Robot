// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice
#include "husky_msgs/msg/detail/husky_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husky_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husky_msgs/msg/detail/husky_status__struct.h"
#include "husky_msgs/msg/detail/husky_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husky_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husky_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husky_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _HuskyStatus__ros_msg_type = husky_msgs__msg__HuskyStatus;

static bool _HuskyStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HuskyStatus__ros_msg_type * ros_message = static_cast<const _HuskyStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: uptime
  {
    cdr << ros_message->uptime;
  }

  // Field name: ros_control_loop_freq
  {
    cdr << ros_message->ros_control_loop_freq;
  }

  // Field name: mcu_and_user_port_current
  {
    cdr << ros_message->mcu_and_user_port_current;
  }

  // Field name: left_driver_current
  {
    cdr << ros_message->left_driver_current;
  }

  // Field name: right_driver_current
  {
    cdr << ros_message->right_driver_current;
  }

  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: left_driver_voltage
  {
    cdr << ros_message->left_driver_voltage;
  }

  // Field name: right_driver_voltage
  {
    cdr << ros_message->right_driver_voltage;
  }

  // Field name: left_driver_temp
  {
    cdr << ros_message->left_driver_temp;
  }

  // Field name: right_driver_temp
  {
    cdr << ros_message->right_driver_temp;
  }

  // Field name: left_motor_temp
  {
    cdr << ros_message->left_motor_temp;
  }

  // Field name: right_motor_temp
  {
    cdr << ros_message->right_motor_temp;
  }

  // Field name: capacity_estimate
  {
    cdr << ros_message->capacity_estimate;
  }

  // Field name: charge_estimate
  {
    cdr << ros_message->charge_estimate;
  }

  // Field name: timeout
  {
    cdr << (ros_message->timeout ? true : false);
  }

  // Field name: lockout
  {
    cdr << (ros_message->lockout ? true : false);
  }

  // Field name: e_stop
  {
    cdr << (ros_message->e_stop ? true : false);
  }

  // Field name: ros_pause
  {
    cdr << (ros_message->ros_pause ? true : false);
  }

  // Field name: no_battery
  {
    cdr << (ros_message->no_battery ? true : false);
  }

  // Field name: current_limit
  {
    cdr << (ros_message->current_limit ? true : false);
  }

  return true;
}

static bool _HuskyStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HuskyStatus__ros_msg_type * ros_message = static_cast<_HuskyStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: uptime
  {
    cdr >> ros_message->uptime;
  }

  // Field name: ros_control_loop_freq
  {
    cdr >> ros_message->ros_control_loop_freq;
  }

  // Field name: mcu_and_user_port_current
  {
    cdr >> ros_message->mcu_and_user_port_current;
  }

  // Field name: left_driver_current
  {
    cdr >> ros_message->left_driver_current;
  }

  // Field name: right_driver_current
  {
    cdr >> ros_message->right_driver_current;
  }

  // Field name: battery_voltage
  {
    cdr >> ros_message->battery_voltage;
  }

  // Field name: left_driver_voltage
  {
    cdr >> ros_message->left_driver_voltage;
  }

  // Field name: right_driver_voltage
  {
    cdr >> ros_message->right_driver_voltage;
  }

  // Field name: left_driver_temp
  {
    cdr >> ros_message->left_driver_temp;
  }

  // Field name: right_driver_temp
  {
    cdr >> ros_message->right_driver_temp;
  }

  // Field name: left_motor_temp
  {
    cdr >> ros_message->left_motor_temp;
  }

  // Field name: right_motor_temp
  {
    cdr >> ros_message->right_motor_temp;
  }

  // Field name: capacity_estimate
  {
    cdr >> ros_message->capacity_estimate;
  }

  // Field name: charge_estimate
  {
    cdr >> ros_message->charge_estimate;
  }

  // Field name: timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->timeout = tmp ? true : false;
  }

  // Field name: lockout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->lockout = tmp ? true : false;
  }

  // Field name: e_stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->e_stop = tmp ? true : false;
  }

  // Field name: ros_pause
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ros_pause = tmp ? true : false;
  }

  // Field name: no_battery
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->no_battery = tmp ? true : false;
  }

  // Field name: current_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->current_limit = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husky_msgs
size_t get_serialized_size_husky_msgs__msg__HuskyStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HuskyStatus__ros_msg_type * ros_message = static_cast<const _HuskyStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name uptime
  {
    size_t item_size = sizeof(ros_message->uptime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ros_control_loop_freq
  {
    size_t item_size = sizeof(ros_message->ros_control_loop_freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mcu_and_user_port_current
  {
    size_t item_size = sizeof(ros_message->mcu_and_user_port_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_driver_current
  {
    size_t item_size = sizeof(ros_message->left_driver_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_driver_current
  {
    size_t item_size = sizeof(ros_message->right_driver_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_driver_voltage
  {
    size_t item_size = sizeof(ros_message->left_driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_driver_voltage
  {
    size_t item_size = sizeof(ros_message->right_driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_driver_temp
  {
    size_t item_size = sizeof(ros_message->left_driver_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_driver_temp
  {
    size_t item_size = sizeof(ros_message->right_driver_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_motor_temp
  {
    size_t item_size = sizeof(ros_message->left_motor_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_motor_temp
  {
    size_t item_size = sizeof(ros_message->right_motor_temp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name capacity_estimate
  {
    size_t item_size = sizeof(ros_message->capacity_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name charge_estimate
  {
    size_t item_size = sizeof(ros_message->charge_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timeout
  {
    size_t item_size = sizeof(ros_message->timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lockout
  {
    size_t item_size = sizeof(ros_message->lockout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name e_stop
  {
    size_t item_size = sizeof(ros_message->e_stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ros_pause
  {
    size_t item_size = sizeof(ros_message->ros_pause);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name no_battery
  {
    size_t item_size = sizeof(ros_message->no_battery);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_limit
  {
    size_t item_size = sizeof(ros_message->current_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _HuskyStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husky_msgs__msg__HuskyStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husky_msgs
size_t max_serialized_size_husky_msgs__msg__HuskyStatus(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: uptime
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ros_control_loop_freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: mcu_and_user_port_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_driver_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_driver_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_driver_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_driver_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_driver_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_driver_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_motor_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_motor_temp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: capacity_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: charge_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: timeout
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: lockout
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: e_stop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ros_pause
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: no_battery
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_limit
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
    using DataType = husky_msgs__msg__HuskyStatus;
    is_plain =
      (
      offsetof(DataType, current_limit) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _HuskyStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husky_msgs__msg__HuskyStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_HuskyStatus = {
  "husky_msgs::msg",
  "HuskyStatus",
  _HuskyStatus__cdr_serialize,
  _HuskyStatus__cdr_deserialize,
  _HuskyStatus__get_serialized_size,
  _HuskyStatus__max_serialized_size
};

static rosidl_message_type_support_t _HuskyStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HuskyStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husky_msgs, msg, HuskyStatus)() {
  return &_HuskyStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
