cmake_minimum_required(VERSION 3.8)
project(eco65b_moveit_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt2
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(eco65b_grasp_interfaces REQUIRED)

add_executable(grasp_moveit_node src/grasp_moveit_node.cpp)
target_compile_features(grasp_moveit_node PRIVATE cxx_std_17)
ament_target_dependencies(grasp_moveit_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
  eco65b_grasp_interfaces
)

install(TARGETS grasp_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
