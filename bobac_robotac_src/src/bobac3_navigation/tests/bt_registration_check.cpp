// Offline registration and XML/port validation. Does not instantiate action nodes.
#include <behaviortree_cpp/bt_factory.h>
#include <tinyxml2.h>
#include <iostream>
#include <stdexcept>
#include <string>

void check_ports(tinyxml2::XMLElement * element, const BT::BehaviorTreeFactory & factory)
{
  const auto manifest = factory.manifests().find(element->Name());
  if (manifest != factory.manifests().end()) {
    for (auto attr = element->FirstAttribute(); attr; attr = attr->Next()) {
      const std::string name = attr->Name();
      if (name != "name" && !manifest->second.ports.count(name)) {
        throw std::runtime_error(std::string(element->Name()) + ": unknown port " + name);
      }
    }
  }
  for (auto child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
    check_ports(child, factory);
  }
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: bt_registration_check BT_XML ROS_LIBRARY_DIR\n";
    return 2;
  }
  try {
    BT::BehaviorTreeFactory factory;
    const char * plugins[] = {
      "nav2_compute_path_to_pose_action_bt_node", "nav2_follow_path_action_bt_node",
      "nav2_clear_costmap_service_bt_node", "nav2_rate_controller_bt_node",
      "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node", "nav2_goal_updated_condition_bt_node",
      "nav2_wait_action_bt_node", "nav2_back_up_action_bt_node",
    };
    for (const auto plugin : plugins) {
      factory.registerFromPlugin(std::string(argv[2]) + "/lib" + plugin + ".so");
    }
    factory.registerBehaviorTreeFromFile(argv[1]);
    tinyxml2::XMLDocument xml;
    if (xml.LoadFile(argv[1]) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error(xml.ErrorStr());
    }
    check_ports(xml.RootElement(), factory);
    std::cout << "PASS: 10 Nav2 BT plugins registered; XML and ports valid. "
              << "No action nodes instantiated or goals sent.\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
