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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib         REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(wetexplorer_hardware REQUIRED)

include_directories(include)

add_executable(imu_covariance_node src/imu_covariance_node.cpp)
add_executable(imu_filtering_node src/imu_filtering_node.cpp)
add_executable(position_controller_node src/position_controller_node.cpp)
add_executable(odometry_tcp_node src/odometry_tcp_node.cpp)
add_executable(goal_publisher src/goal_publisher.cpp)
add_executable(map_server_node src/map_server_node.cpp)
add_executable(gps_noise_node src/gps_noise.cpp)
add_executable(ground_truth_node src/ground_truth.cpp)
add_executable(error_display_node src/error_display.cpp)
#add_executable(dummy_map_node src/dummy_map.cpp)
add_executable(ring_path_planner src/ring_path_planner.cpp)

add_executable(6d_pose_caller src/6d_pose_caller.cpp)

ament_target_dependencies(6d_pose_caller rclcpp rclcpp_action sensor_msgs std_srvs)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTCP.action"
  "action/LocalizeObject.action"
  "action/UpdateMap.action"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)



ament_export_dependencies(rosidl_default_runtime)

# Allow including the generated action headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})


add_library(ring_layer_plugin SHARED
  src/ring_layer.cpp
)

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



ament_target_dependencies(position_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  std_srvs  
  rclcpp_action
)
# Link the generated action interface into your node
rosidl_target_interfaces(position_controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(ring_path_planner ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(map_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(6d_pose_caller ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(odometry_tcp_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

ament_target_dependencies(goal_publisher
  rclcpp
  std_msgs  
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(map_server_node
  rclcpp
  std_msgs  
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
)

#ament_target_dependencies(dummy_map_node
#  rclcpp
#  visualization_msgs
#  geometry_msgs
#)

ament_target_dependencies(gps_noise_node
  rclcpp
  sensor_msgs  
)

ament_target_dependencies(ground_truth_node
  rclcpp
  sensor_msgs  
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

ament_target_dependencies(error_display_node
  rclcpp
  sensor_msgs 
  nav_msgs
)

ament_target_dependencies(ring_path_planner
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  visualization_msgs
  lifecycle_msgs
  rclcpp_action
  nav2_msgs
  std_srvs
  sensor_msgs
  wetexplorer_hardware
)

ament_target_dependencies(ring_layer_plugin
  rclcpp
  visualization_msgs
  nav2_costmap_2d
  nav_msgs
  pluginlib  
)

install(DIRECTORY launch config action
  DESTINATION share/${PROJECT_NAME}
)




install(
  TARGETS
    imu_covariance_node  
    imu_filtering_node
    position_controller_node
    odometry_tcp_node
    goal_publisher
    map_server_node
    gps_noise_node
    ground_truth_node
    error_display_node
    ring_path_planner
    ring_layer_plugin
    6d_pose_caller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(nav2_costmap_2d ring_layer.xml)
ament_export_libraries(ring_layer_plugin)
ament_package()
