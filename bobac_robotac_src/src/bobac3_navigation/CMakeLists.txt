cmake_minimum_required(VERSION 3.5)
project(bobac3_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz2
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(PROGRAMS
  scripts/laser_qos_relay.py
  scripts/navigation_readiness.py
  scripts/analyze_transport_trace.py
  scripts/check_path_clearance.py
  scripts/generate_world_map.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
