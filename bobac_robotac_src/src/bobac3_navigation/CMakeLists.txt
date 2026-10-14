cmake_minimum_required(VERSION 3.5)
project(bobac3_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz2
  behavior_trees
  DESTINATION share/${PROJECT_NAME})

ament_package()
