#include "bobac3_navigation/connected_path.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bobac3_navigation {
// Opt-in wrapper; retain Jazzy Smac search, parameters and lifecycle implementation.
class ConnectedSmacPlanner2D : public nav2_smac_planner::SmacPlanner2D {
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    auto raw=SmacPlanner2D::createPlan(start, goal, cancel_checker);
    if (raw.poses.empty()) return raw;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    nav_msgs::msg::Path path;
    auto free_segment=[&](const auto & start_pose,const auto & end_pose) {
      const auto & a=start_pose.pose.position; const auto & b=end_pose.pose.position;
      const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.005)));
      for (int i=0;i<=steps;++i) {
        if (cancel_checker()) throw nav2_core::PlannerCancelled("endpoint check canceled");
        const double t=static_cast<double>(i)/steps;
        unsigned int mx,my;
        if (!_costmap->worldToMap(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),mx,my) ||
            _costmap->getCost(mx,my)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
      }
      return true;
    };
    try { path=connectEndpointsAndDensify(raw, start, goal, _costmap->getResolution(),free_segment); }
    catch (const std::invalid_argument & e) {
      throw nav2_core::NoValidPathCouldBeFound(e.what());
    }
    for (const auto & pose : path.poses) {
      if (cancel_checker()) throw nav2_core::PlannerCancelled("connected plan canceled");
      unsigned int mx,my;
      const auto & p=pose.pose.position;
      if (!_costmap->worldToMap(p.x,p.y,mx,my) ||
          _costmap->getCost(mx,my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        throw nav2_core::NoValidPathCouldBeFound("connected path enters blocked/unknown cell");
      }
    }
    // A must still apply the independent whole-polyline >=0.55m clearance gate.
    return path;
  }
};
}  // namespace bobac3_navigation
PLUGINLIB_EXPORT_CLASS(bobac3_navigation::ConnectedSmacPlanner2D, nav2_core::GlobalPlanner)
