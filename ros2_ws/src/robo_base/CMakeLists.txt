cmake_minimum_required(VERSION 3.8)
project(robo_base)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robo_io REQUIRED)
# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoboIO.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# Declare executables
add_executable(robo_base_ros2 src/RoboteqDevice.cpp src/robo_base.cpp)
ament_target_dependencies(robo_base_ros2 rclcpp std_msgs geometry_msgs robo_io)
#rosidl_target_interfaces(robo_base_ros2 ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_include_directories(robo_base_ros2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

#add_executable(robo_plan src/robo_plan.cpp)
#ament_target_dependencies(robo_plan rclcpp std_msgs geometry_msgs robo_io)
#rosidl_target_interfaces(robo_plan ${PROJECT_NAME} "rosidl_typesupport_cpp")

#add_executable(teleop_joy src/teleop.cpp)
#ament_target_dependencies(teleop_joy rclcpp std_msgs geometry_msgs)
#rosidl_target_interfaces(teleop_joy ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mode src/RoboteqDevice.cpp src/robo_enable.cpp)
ament_target_dependencies(mode rclcpp std_msgs geometry_msgs robo_io)
#rosidl_target_interfaces(mode ${PROJECT_NAME} "rosidl_typesupport_cpp")

#add_executable(robo_base_laser src/laser.cpp)
#ament_target_dependencies(robo_base_laser rclcpp sensor_msgs geometry_msgs)
#rosidl_target_interfaces(robo_base_laser ${PROJECT_NAME} "rosidl_typesupport_cpp")

#add_executable(robo_base_gps src/gps_rtk.cpp)
#ament_target_dependencies(robo_base_gps rclcpp std_msgs geometry_msgs)
#rosidl_target_interfaces(robo_base_gps ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  robo_base_ros2
  mode  
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

target_link_libraries(robo_base_ros2
    ${rclcpp_LIBRARIES}   
    ${std_msgs_LIBRARIES}
    ${robo_io_LIBRARIES} # Link against custom_interfaces   
)


# Package export
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs)
ament_package()
