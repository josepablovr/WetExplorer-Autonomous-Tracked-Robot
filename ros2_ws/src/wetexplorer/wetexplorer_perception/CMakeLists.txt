cmake_minimum_required(VERSION 3.8)
project(wetexplorer_perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yolo_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wetexplorer_navigation REQUIRED)  # for the action


add_executable(object_depth_cloud_node src/object_depth_cloud_node.cpp)
add_executable(object_localizer src/object_localizer.cpp)
add_executable(rgbd_map_node src/rgbd_map_node.cpp)

ament_target_dependencies(object_depth_cloud_node
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  cv_bridge
  yolo_msgs
  OpenCV
)

ament_target_dependencies(rgbd_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  cv_bridge
  OpenCV
  tf2
  tf2_ros
)

ament_target_dependencies(object_localizer
  rclcpp rclcpp_action sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros wetexplorer_navigation)


install(TARGETS object_depth_cloud_node rgbd_map_node object_localizer
  DESTINATION lib/${PROJECT_NAME})

# (Optional) install params/launch files if you add them later
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
