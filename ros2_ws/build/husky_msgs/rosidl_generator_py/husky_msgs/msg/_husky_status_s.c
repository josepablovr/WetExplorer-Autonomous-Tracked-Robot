// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "husky_msgs/msg/detail/husky_status__struct.h"
#include "husky_msgs/msg/detail/husky_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool husky_msgs__msg__husky_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("husky_msgs.msg._husky_status.HuskyStatus", full_classname_dest, 40) == 0);
  }
  husky_msgs__msg__HuskyStatus * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // uptime
    PyObject * field = PyObject_GetAttrString(_pymsg, "uptime");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->uptime = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ros_control_loop_freq
    PyObject * field = PyObject_GetAttrString(_pymsg, "ros_control_loop_freq");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ros_control_loop_freq = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mcu_and_user_port_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "mcu_and_user_port_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mcu_and_user_port_current = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_driver_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_driver_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_driver_current = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_driver_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_driver_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_driver_current = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_driver_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_driver_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_driver_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_driver_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_driver_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_driver_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_driver_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_driver_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_driver_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_driver_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_driver_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_driver_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_motor_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_motor_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_motor_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_motor_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_motor_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_motor_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // capacity_estimate
    PyObject * field = PyObject_GetAttrString(_pymsg, "capacity_estimate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->capacity_estimate = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // charge_estimate
    PyObject * field = PyObject_GetAttrString(_pymsg, "charge_estimate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->charge_estimate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // timeout
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeout");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->timeout = (Py_True == field);
    Py_DECREF(field);
  }
  {  // lockout
    PyObject * field = PyObject_GetAttrString(_pymsg, "lockout");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->lockout = (Py_True == field);
    Py_DECREF(field);
  }
  {  // e_stop
    PyObject * field = PyObject_GetAttrString(_pymsg, "e_stop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->e_stop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ros_pause
    PyObject * field = PyObject_GetAttrString(_pymsg, "ros_pause");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ros_pause = (Py_True == field);
    Py_DECREF(field);
  }
  {  // no_battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "no_battery");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->no_battery = (Py_True == field);
    Py_DECREF(field);
  }
  {  // current_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->current_limit = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husky_msgs__msg__husky_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HuskyStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husky_msgs.msg._husky_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HuskyStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husky_msgs__msg__HuskyStatus * ros_message = (husky_msgs__msg__HuskyStatus *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // uptime
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->uptime);
    {
      int rc = PyObject_SetAttrString(_pymessage, "uptime", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ros_control_loop_freq
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ros_control_loop_freq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ros_control_loop_freq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mcu_and_user_port_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mcu_and_user_port_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mcu_and_user_port_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_driver_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_driver_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_driver_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_driver_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_driver_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_driver_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_driver_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_driver_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_driver_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_driver_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_driver_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_driver_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_driver_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_driver_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_driver_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_driver_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_driver_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_driver_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_motor_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_motor_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_motor_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_motor_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_motor_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_motor_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // capacity_estimate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->capacity_estimate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "capacity_estimate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charge_estimate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->charge_estimate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charge_estimate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeout
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->timeout ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeout", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lockout
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->lockout ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lockout", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // e_stop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->e_stop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "e_stop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ros_pause
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ros_pause ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ros_pause", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // no_battery
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->no_battery ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "no_battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->current_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
