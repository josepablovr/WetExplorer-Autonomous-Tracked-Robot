cmake_minimum_required(VERSION 3.8)
project(wetexplorer_control)

# Ensure we support C++14 or above (if needed)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find needed packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


############
# BUILD NODES
############

# 1) Forward Kinematics Node
add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)
ament_target_dependencies(forward_kinematics_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)


add_executable(forward_kinematics_sim src/forward_kinematics_sim.cpp)
ament_target_dependencies(forward_kinematics_sim
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)


# 2) Robot Control Node
add_executable(robot_control_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(robot_control_node
  rclcpp
  std_msgs
  geometry_msgs
)

# 3) Safe Commands Node
add_executable(safe_commands_node src/safe_commands_node.cpp)
ament_target_dependencies(safe_commands_node
  rclcpp
  geometry_msgs
)

########################
# INSTALL TARGETS
########################

install(TARGETS
  forward_kinematics_node
  forward_kinematics_sim
  robot_control_node
  safe_commands_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
