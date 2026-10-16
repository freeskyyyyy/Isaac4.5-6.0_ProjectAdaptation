cmake_minimum_required(VERSION 3.5)
project(bobac3_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
add_executable(sim_time_velocity_smoother src/sim_time_velocity_smoother.cpp)
target_compile_features(sim_time_velocity_smoother PRIVATE cxx_std_17)
target_include_directories(sim_time_velocity_smoother PRIVATE include)
ament_target_dependencies(sim_time_velocity_smoother nav2_velocity_smoother)
install(TARGETS sim_time_velocity_smoother DESTINATION lib/${PROJECT_NAME})
add_library(connected_smac_planner SHARED src/connected_smac_planner.cpp)
target_compile_features(connected_smac_planner PUBLIC cxx_std_17)
target_include_directories(connected_smac_planner PRIVATE include)
ament_target_dependencies(connected_smac_planner nav2_smac_planner nav2_core nav2_costmap_2d nav_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)
install(TARGETS connected_smac_planner LIBRARY DESTINATION lib)
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(suite test_navigation_config test_path_clearance test_world_scene_map test_world_transport_control test_control_feedback)
    add_test(NAME ${suite} COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/${suite}.py)
    set_tests_properties(${suite} PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  endforeach()
  add_executable(feedback_step_check tests/feedback_step_check.cpp)
  target_compile_features(feedback_step_check PRIVATE cxx_std_17)
  target_include_directories(feedback_step_check PRIVATE include)
  add_test(NAME feedback_step_check COMMAND feedback_step_check)
  add_executable(connected_path_check tests/connected_path_check.cpp)
  target_compile_features(connected_path_check PRIVATE cxx_std_17)
  target_include_directories(connected_path_check PRIVATE include)
  ament_target_dependencies(connected_path_check nav_msgs)
  add_test(NAME connected_path_check COMMAND connected_path_check)
  add_executable(planner_plugin_load_check tests/planner_plugin_load_check.cpp)
  target_compile_features(planner_plugin_load_check PRIVATE cxx_std_17)
  ament_target_dependencies(planner_plugin_load_check nav2_core pluginlib)
  add_test(NAME planner_plugin_load_check COMMAND planner_plugin_load_check)
endif()

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
  scripts/analyze_control_feedback.py
  scripts/analyze_transport_trace.py
  scripts/check_path_clearance.py
  scripts/generate_world_map.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
