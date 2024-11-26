// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice
#include "husky_msgs/msg/detail/husky_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
husky_msgs__msg__HuskyStatus__init(husky_msgs__msg__HuskyStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    husky_msgs__msg__HuskyStatus__fini(msg);
    return false;
  }
  // uptime
  // ros_control_loop_freq
  // mcu_and_user_port_current
  // left_driver_current
  // right_driver_current
  // battery_voltage
  // left_driver_voltage
  // right_driver_voltage
  // left_driver_temp
  // right_driver_temp
  // left_motor_temp
  // right_motor_temp
  // capacity_estimate
  // charge_estimate
  // timeout
  // lockout
  // e_stop
  // ros_pause
  // no_battery
  // current_limit
  return true;
}

void
husky_msgs__msg__HuskyStatus__fini(husky_msgs__msg__HuskyStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // uptime
  // ros_control_loop_freq
  // mcu_and_user_port_current
  // left_driver_current
  // right_driver_current
  // battery_voltage
  // left_driver_voltage
  // right_driver_voltage
  // left_driver_temp
  // right_driver_temp
  // left_motor_temp
  // right_motor_temp
  // capacity_estimate
  // charge_estimate
  // timeout
  // lockout
  // e_stop
  // ros_pause
  // no_battery
  // current_limit
}

bool
husky_msgs__msg__HuskyStatus__are_equal(const husky_msgs__msg__HuskyStatus * lhs, const husky_msgs__msg__HuskyStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // uptime
  if (lhs->uptime != rhs->uptime) {
    return false;
  }
  // ros_control_loop_freq
  if (lhs->ros_control_loop_freq != rhs->ros_control_loop_freq) {
    return false;
  }
  // mcu_and_user_port_current
  if (lhs->mcu_and_user_port_current != rhs->mcu_and_user_port_current) {
    return false;
  }
  // left_driver_current
  if (lhs->left_driver_current != rhs->left_driver_current) {
    return false;
  }
  // right_driver_current
  if (lhs->right_driver_current != rhs->right_driver_current) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // left_driver_voltage
  if (lhs->left_driver_voltage != rhs->left_driver_voltage) {
    return false;
  }
  // right_driver_voltage
  if (lhs->right_driver_voltage != rhs->right_driver_voltage) {
    return false;
  }
  // left_driver_temp
  if (lhs->left_driver_temp != rhs->left_driver_temp) {
    return false;
  }
  // right_driver_temp
  if (lhs->right_driver_temp != rhs->right_driver_temp) {
    return false;
  }
  // left_motor_temp
  if (lhs->left_motor_temp != rhs->left_motor_temp) {
    return false;
  }
  // right_motor_temp
  if (lhs->right_motor_temp != rhs->right_motor_temp) {
    return false;
  }
  // capacity_estimate
  if (lhs->capacity_estimate != rhs->capacity_estimate) {
    return false;
  }
  // charge_estimate
  if (lhs->charge_estimate != rhs->charge_estimate) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // lockout
  if (lhs->lockout != rhs->lockout) {
    return false;
  }
  // e_stop
  if (lhs->e_stop != rhs->e_stop) {
    return false;
  }
  // ros_pause
  if (lhs->ros_pause != rhs->ros_pause) {
    return false;
  }
  // no_battery
  if (lhs->no_battery != rhs->no_battery) {
    return false;
  }
  // current_limit
  if (lhs->current_limit != rhs->current_limit) {
    return false;
  }
  return true;
}

bool
husky_msgs__msg__HuskyStatus__copy(
  const husky_msgs__msg__HuskyStatus * input,
  husky_msgs__msg__HuskyStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // uptime
  output->uptime = input->uptime;
  // ros_control_loop_freq
  output->ros_control_loop_freq = input->ros_control_loop_freq;
  // mcu_and_user_port_current
  output->mcu_and_user_port_current = input->mcu_and_user_port_current;
  // left_driver_current
  output->left_driver_current = input->left_driver_current;
  // right_driver_current
  output->right_driver_current = input->right_driver_current;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // left_driver_voltage
  output->left_driver_voltage = input->left_driver_voltage;
  // right_driver_voltage
  output->right_driver_voltage = input->right_driver_voltage;
  // left_driver_temp
  output->left_driver_temp = input->left_driver_temp;
  // right_driver_temp
  output->right_driver_temp = input->right_driver_temp;
  // left_motor_temp
  output->left_motor_temp = input->left_motor_temp;
  // right_motor_temp
  output->right_motor_temp = input->right_motor_temp;
  // capacity_estimate
  output->capacity_estimate = input->capacity_estimate;
  // charge_estimate
  output->charge_estimate = input->charge_estimate;
  // timeout
  output->timeout = input->timeout;
  // lockout
  output->lockout = input->lockout;
  // e_stop
  output->e_stop = input->e_stop;
  // ros_pause
  output->ros_pause = input->ros_pause;
  // no_battery
  output->no_battery = input->no_battery;
  // current_limit
  output->current_limit = input->current_limit;
  return true;
}

husky_msgs__msg__HuskyStatus *
husky_msgs__msg__HuskyStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husky_msgs__msg__HuskyStatus * msg = (husky_msgs__msg__HuskyStatus *)allocator.allocate(sizeof(husky_msgs__msg__HuskyStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husky_msgs__msg__HuskyStatus));
  bool success = husky_msgs__msg__HuskyStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husky_msgs__msg__HuskyStatus__destroy(husky_msgs__msg__HuskyStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husky_msgs__msg__HuskyStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husky_msgs__msg__HuskyStatus__Sequence__init(husky_msgs__msg__HuskyStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husky_msgs__msg__HuskyStatus * data = NULL;

  if (size) {
    data = (husky_msgs__msg__HuskyStatus *)allocator.zero_allocate(size, sizeof(husky_msgs__msg__HuskyStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husky_msgs__msg__HuskyStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husky_msgs__msg__HuskyStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
husky_msgs__msg__HuskyStatus__Sequence__fini(husky_msgs__msg__HuskyStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      husky_msgs__msg__HuskyStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

husky_msgs__msg__HuskyStatus__Sequence *
husky_msgs__msg__HuskyStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husky_msgs__msg__HuskyStatus__Sequence * array = (husky_msgs__msg__HuskyStatus__Sequence *)allocator.allocate(sizeof(husky_msgs__msg__HuskyStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husky_msgs__msg__HuskyStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husky_msgs__msg__HuskyStatus__Sequence__destroy(husky_msgs__msg__HuskyStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husky_msgs__msg__HuskyStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husky_msgs__msg__HuskyStatus__Sequence__are_equal(const husky_msgs__msg__HuskyStatus__Sequence * lhs, const husky_msgs__msg__HuskyStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husky_msgs__msg__HuskyStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husky_msgs__msg__HuskyStatus__Sequence__copy(
  const husky_msgs__msg__HuskyStatus__Sequence * input,
  husky_msgs__msg__HuskyStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husky_msgs__msg__HuskyStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husky_msgs__msg__HuskyStatus * data =
      (husky_msgs__msg__HuskyStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husky_msgs__msg__HuskyStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husky_msgs__msg__HuskyStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husky_msgs__msg__HuskyStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
