cmake_minimum_required(VERSION 3.5)
project(wetexplorer_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# IMU Covariance Node
add_executable(imu_covariance_node src/imu_covariance_node.cpp)
add_executable(imu_filtering_node src/imu_filtering_node.cpp)
ament_target_dependencies(imu_covariance_node
  rclcpp
  std_msgs  
  geometry_msgs
  sensor_msgs
  
)

ament_target_dependencies(imu_filtering_node
  rclcpp
  std_msgs  
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  imu_covariance_node  
  imu_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()