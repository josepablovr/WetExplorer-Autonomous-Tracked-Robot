cmake_minimum_required(VERSION 3.8)
project(wetexplorer_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LiftingJointStatus.msg"
  "action/Calibrate.action"
  "action/MoveJoint.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Build executable
add_executable(lifting_joint_interface src/lifting_joint_interface.cpp src/serial_device.cpp)

ament_target_dependencies(lifting_joint_interface
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
)

# Include directories
include_directories(
  include
)

# Link generated interfaces to the executable
rosidl_target_interfaces(lifting_joint_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install include files (headers)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install executable
install(TARGETS lifting_joint_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install interface and launch files
install(DIRECTORY action msg launch
  DESTINATION share/${PROJECT_NAME}
)

# Linting setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
