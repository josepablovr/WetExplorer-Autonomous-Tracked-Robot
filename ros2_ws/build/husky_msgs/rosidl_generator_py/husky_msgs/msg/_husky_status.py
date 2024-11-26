# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husky_msgs:msg/HuskyStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HuskyStatus(type):
    """Metaclass of message 'HuskyStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('husky_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'husky_msgs.msg.HuskyStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__husky_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__husky_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__husky_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__husky_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__husky_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HuskyStatus(metaclass=Metaclass_HuskyStatus):
    """Message class 'HuskyStatus'."""

    __slots__ = [
        '_header',
        '_uptime',
        '_ros_control_loop_freq',
        '_mcu_and_user_port_current',
        '_left_driver_current',
        '_right_driver_current',
        '_battery_voltage',
        '_left_driver_voltage',
        '_right_driver_voltage',
        '_left_driver_temp',
        '_right_driver_temp',
        '_left_motor_temp',
        '_right_motor_temp',
        '_capacity_estimate',
        '_charge_estimate',
        '_timeout',
        '_lockout',
        '_e_stop',
        '_ros_pause',
        '_no_battery',
        '_current_limit',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'uptime': 'uint32',
        'ros_control_loop_freq': 'double',
        'mcu_and_user_port_current': 'double',
        'left_driver_current': 'double',
        'right_driver_current': 'double',
        'battery_voltage': 'double',
        'left_driver_voltage': 'double',
        'right_driver_voltage': 'double',
        'left_driver_temp': 'double',
        'right_driver_temp': 'double',
        'left_motor_temp': 'double',
        'right_motor_temp': 'double',
        'capacity_estimate': 'uint16',
        'charge_estimate': 'double',
        'timeout': 'boolean',
        'lockout': 'boolean',
        'e_stop': 'boolean',
        'ros_pause': 'boolean',
        'no_battery': 'boolean',
        'current_limit': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.uptime = kwargs.get('uptime', int())
        self.ros_control_loop_freq = kwargs.get('ros_control_loop_freq', float())
        self.mcu_and_user_port_current = kwargs.get('mcu_and_user_port_current', float())
        self.left_driver_current = kwargs.get('left_driver_current', float())
        self.right_driver_current = kwargs.get('right_driver_current', float())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.left_driver_voltage = kwargs.get('left_driver_voltage', float())
        self.right_driver_voltage = kwargs.get('right_driver_voltage', float())
        self.left_driver_temp = kwargs.get('left_driver_temp', float())
        self.right_driver_temp = kwargs.get('right_driver_temp', float())
        self.left_motor_temp = kwargs.get('left_motor_temp', float())
        self.right_motor_temp = kwargs.get('right_motor_temp', float())
        self.capacity_estimate = kwargs.get('capacity_estimate', int())
        self.charge_estimate = kwargs.get('charge_estimate', float())
        self.timeout = kwargs.get('timeout', bool())
        self.lockout = kwargs.get('lockout', bool())
        self.e_stop = kwargs.get('e_stop', bool())
        self.ros_pause = kwargs.get('ros_pause', bool())
        self.no_battery = kwargs.get('no_battery', bool())
        self.current_limit = kwargs.get('current_limit', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.uptime != other.uptime:
            return False
        if self.ros_control_loop_freq != other.ros_control_loop_freq:
            return False
        if self.mcu_and_user_port_current != other.mcu_and_user_port_current:
            return False
        if self.left_driver_current != other.left_driver_current:
            return False
        if self.right_driver_current != other.right_driver_current:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.left_driver_voltage != other.left_driver_voltage:
            return False
        if self.right_driver_voltage != other.right_driver_voltage:
            return False
        if self.left_driver_temp != other.left_driver_temp:
            return False
        if self.right_driver_temp != other.right_driver_temp:
            return False
        if self.left_motor_temp != other.left_motor_temp:
            return False
        if self.right_motor_temp != other.right_motor_temp:
            return False
        if self.capacity_estimate != other.capacity_estimate:
            return False
        if self.charge_estimate != other.charge_estimate:
            return False
        if self.timeout != other.timeout:
            return False
        if self.lockout != other.lockout:
            return False
        if self.e_stop != other.e_stop:
            return False
        if self.ros_pause != other.ros_pause:
            return False
        if self.no_battery != other.no_battery:
            return False
        if self.current_limit != other.current_limit:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def uptime(self):
        """Message field 'uptime'."""
        return self._uptime

    @uptime.setter
    def uptime(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'uptime' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'uptime' field must be an unsigned integer in [0, 4294967295]"
        self._uptime = value

    @builtins.property
    def ros_control_loop_freq(self):
        """Message field 'ros_control_loop_freq'."""
        return self._ros_control_loop_freq

    @ros_control_loop_freq.setter
    def ros_control_loop_freq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ros_control_loop_freq' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ros_control_loop_freq' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ros_control_loop_freq = value

    @builtins.property
    def mcu_and_user_port_current(self):
        """Message field 'mcu_and_user_port_current'."""
        return self._mcu_and_user_port_current

    @mcu_and_user_port_current.setter
    def mcu_and_user_port_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mcu_and_user_port_current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mcu_and_user_port_current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mcu_and_user_port_current = value

    @builtins.property
    def left_driver_current(self):
        """Message field 'left_driver_current'."""
        return self._left_driver_current

    @left_driver_current.setter
    def left_driver_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_driver_current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_driver_current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_driver_current = value

    @builtins.property
    def right_driver_current(self):
        """Message field 'right_driver_current'."""
        return self._right_driver_current

    @right_driver_current.setter
    def right_driver_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_driver_current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_driver_current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_driver_current = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_voltage = value

    @builtins.property
    def left_driver_voltage(self):
        """Message field 'left_driver_voltage'."""
        return self._left_driver_voltage

    @left_driver_voltage.setter
    def left_driver_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_driver_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_driver_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_driver_voltage = value

    @builtins.property
    def right_driver_voltage(self):
        """Message field 'right_driver_voltage'."""
        return self._right_driver_voltage

    @right_driver_voltage.setter
    def right_driver_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_driver_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_driver_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_driver_voltage = value

    @builtins.property
    def left_driver_temp(self):
        """Message field 'left_driver_temp'."""
        return self._left_driver_temp

    @left_driver_temp.setter
    def left_driver_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_driver_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_driver_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_driver_temp = value

    @builtins.property
    def right_driver_temp(self):
        """Message field 'right_driver_temp'."""
        return self._right_driver_temp

    @right_driver_temp.setter
    def right_driver_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_driver_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_driver_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_driver_temp = value

    @builtins.property
    def left_motor_temp(self):
        """Message field 'left_motor_temp'."""
        return self._left_motor_temp

    @left_motor_temp.setter
    def left_motor_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_motor_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_motor_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_motor_temp = value

    @builtins.property
    def right_motor_temp(self):
        """Message field 'right_motor_temp'."""
        return self._right_motor_temp

    @right_motor_temp.setter
    def right_motor_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_motor_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_motor_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_motor_temp = value

    @builtins.property
    def capacity_estimate(self):
        """Message field 'capacity_estimate'."""
        return self._capacity_estimate

    @capacity_estimate.setter
    def capacity_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'capacity_estimate' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'capacity_estimate' field must be an unsigned integer in [0, 65535]"
        self._capacity_estimate = value

    @builtins.property
    def charge_estimate(self):
        """Message field 'charge_estimate'."""
        return self._charge_estimate

    @charge_estimate.setter
    def charge_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charge_estimate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'charge_estimate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._charge_estimate = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'timeout' field must be of type 'bool'"
        self._timeout = value

    @builtins.property
    def lockout(self):
        """Message field 'lockout'."""
        return self._lockout

    @lockout.setter
    def lockout(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'lockout' field must be of type 'bool'"
        self._lockout = value

    @builtins.property
    def e_stop(self):
        """Message field 'e_stop'."""
        return self._e_stop

    @e_stop.setter
    def e_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'e_stop' field must be of type 'bool'"
        self._e_stop = value

    @builtins.property
    def ros_pause(self):
        """Message field 'ros_pause'."""
        return self._ros_pause

    @ros_pause.setter
    def ros_pause(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ros_pause' field must be of type 'bool'"
        self._ros_pause = value

    @builtins.property
    def no_battery(self):
        """Message field 'no_battery'."""
        return self._no_battery

    @no_battery.setter
    def no_battery(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'no_battery' field must be of type 'bool'"
        self._no_battery = value

    @builtins.property
    def current_limit(self):
        """Message field 'current_limit'."""
        return self._current_limit

    @current_limit.setter
    def current_limit(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'current_limit' field must be of type 'bool'"
        self._current_limit = value
