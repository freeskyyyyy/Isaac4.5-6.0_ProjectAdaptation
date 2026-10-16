// Load the installed plugin only. No ROS initialization, nodes, configure or planning.
#include <iostream>
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"
int main() {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  auto plugin=loader.createUniqueInstance("bobac3_navigation::ConnectedSmacPlanner2D");
  if (!plugin) return 1;
  std::cout << "PASS: installed ConnectedSmacPlanner2D resolves and loads without ROS nodes\n";
}
