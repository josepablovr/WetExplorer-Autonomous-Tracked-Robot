#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rtcm_msgs::rtcm_msgs__rosidl_generator_py" for configuration ""
set_property(TARGET rtcm_msgs::rtcm_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rtcm_msgs::rtcm_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librtcm_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "librtcm_msgs__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rtcm_msgs::rtcm_msgs__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_rtcm_msgs::rtcm_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/librtcm_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
