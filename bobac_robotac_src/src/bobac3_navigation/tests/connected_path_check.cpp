// Pure geometry regression. No ROS initialization, nodes or commands.
#include <stdexcept>
#include <cmath>
#include <iostream>
#include "bobac3_navigation/connected_path.hpp"
void require(bool value) { if (!value) throw std::runtime_error("navigation offline regression failed"); }
int main() {
  nav_msgs::msg::Path raw; raw.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped start; start.header = raw.header;
  start.pose.position.x = 8; start.pose.position.y = 1.082; start.pose.orientation.w = 1;
  for (auto xy : {std::pair{7.95,1.05}, {7.9,1.1}, {7.85,1.15}}) {
    auto p = start; p.pose.position.x = xy.first; p.pose.position.y = xy.second;
    raw.poses.push_back(p);
  }
  auto before = raw;
  auto path = bobac3_navigation::connectAndDensify(raw, start, .05);
  require(raw == before);
  require(path.poses.front() == start);
  require(path.poses.back() == raw.poses.back());
  size_t original_vertex = 0;
  for (size_t i=1; i<path.poses.size(); ++i) {
    const auto & a=path.poses[i-1].pose.position; const auto & b=path.poses[i].pose.position;
    require(std::hypot(b.x-a.x,b.y-a.y) <= .010000001);
    if (original_vertex<raw.poses.size() && path.poses[i] == raw.poses[original_vertex]) ++original_vertex;
    // Every new point remains on the original connector/polyline, no corner cutting.
    double best = 1e9;
    auto prev = start.pose.position;
    for (const auto & q : raw.poses) {
      auto end = q.pose.position;
      double dx=end.x-prev.x,dy=end.y-prev.y,den=dx*dx+dy*dy;
      double t=den ? std::clamp(((b.x-prev.x)*dx+(b.y-prev.y)*dy)/den,0.,1.) : 0.;
      best=std::min(best,std::hypot(b.x-prev.x-t*dx,b.y-prev.y-t*dy)); prev=end;
    }
    require(best < 1e-12);
  }
  require(original_vertex == raw.poses.size());
  auto wrong=start; wrong.header.frame_id="odom";
  bool rejected=false; try {bobac3_navigation::connectAndDensify(raw,wrong,.05);} catch (const std::invalid_argument &) {rejected=true;}
  require(rejected);
  wrong=start;wrong.pose.position.x+=1;rejected=false;
  try {bobac3_navigation::connectAndDensify(raw,wrong,.05);} catch (const std::invalid_argument &) {rejected=true;}
  require(rejected);
  wrong=start;wrong.pose.position.x=NAN;rejected=false;
  try {bobac3_navigation::connectAndDensify(raw,wrong,.05);} catch (const std::invalid_argument &) {rejected=true;}
  require(rejected);
  require(bobac3_navigation::connectAndDensify(nav_msgs::msg::Path{},start,.05).poses.empty());
  // Real short-goal counterexample: the first cell is behind the measured start,
  // and the last cell is 59mm from the requested goal. Free local connectors
  // must remove the initial backtrack and retain the exact goal orientation.
  nav_msgs::msg::Path east; east.header=raw.header;
  for (double x : {7.95,8.,8.05,8.10,8.15,8.20,8.25}) {
    auto p=start; p.pose.position.x=x; p.pose.position.y=1.05; east.poses.push_back(p);
  }
  const auto east_before=east;
  auto goal=start; goal.pose.position.x=8.30; goal.pose.orientation.z=.5;
  goal.pose.orientation.w=std::sqrt(.75);
  auto free=[](const auto &,const auto &) { return true; };
  auto fixed=bobac3_navigation::connectEndpointsAndDensify(east,start,goal,.05,free);
  require(east==east_before && fixed.poses.front()==start && fixed.poses.back()==goal);
  for (size_t i=1;i<fixed.poses.size();++i) {
    const auto & a=fixed.poses[i-1].pose.position; const auto & b=fixed.poses[i].pose.position;
    require(b.x>=a.x-1e-12 && std::hypot(b.x-a.x,b.y-a.y)<=.010000001);
  }
  // A local corner cannot be cut merely because it is within one cell diagonal.
  auto obstructed=[&](const auto & a,const auto & b) {
    return !(a==start && b.pose.position.x>=8.);
  };
  auto retained=bobac3_navigation::connectEndpointsAndDensify(east,start,goal,.05,obstructed);
  require(std::any_of(retained.poses.begin(),retained.poses.end(),
    [&](const auto & p) { return p==east.poses.front(); }));
  auto blocked_goal=[&](const auto &,const auto & b) { return b!=goal; };
  rejected=false;
  try {bobac3_navigation::connectEndpointsAndDensify(east,start,goal,.05,blocked_goal);}
  catch (const std::invalid_argument &) {rejected=true;}
  require(rejected);
  auto far_goal=goal; far_goal.pose.position.x+=.10; rejected=false;
  try {bobac3_navigation::connectEndpointsAndDensify(east,start,far_goal,.05,free);}
  catch (const std::invalid_argument &) {rejected=true;}
  require(rejected);
  // Real long-route prefix: selecting only whole vertices inside one grid
  // diagonal connects southwest before the route heads northwest. A short RPP
  // lookahead then repeatedly turns toward this quantization artifact.
  auto measured=start; measured.pose.position.x=8.035802353055104;
  measured.pose.position.y=2.035197600083592;
  nav_msgs::msg::Path northwest; northwest.header=raw.header;
  for (auto xy : {std::pair{8.05,2.0}, {8.,2.0}, {7.95,2.05}, {7.9,2.1}}) {
    auto p=measured; p.pose.position.x=xy.first; p.pose.position.y=xy.second;
    northwest.poses.push_back(p);
  }
  auto northwest_goal=northwest.poses.back();
  const auto northwest_before=northwest;
  auto connected=bobac3_navigation::connectEndpointsAndDensify(
    northwest,measured,northwest_goal,.05,free);
  const auto & join=connected.poses[1].pose.position;
  double initial_angle=std::atan2(join.y-measured.pose.position.y,join.x-measured.pose.position.x);
  require(std::abs(std::atan2(std::sin(initial_angle-M_PI),std::cos(initial_angle-M_PI)))<.15);
  require(connected.poses.front()==measured && connected.poses.back()==northwest_goal);
  require(northwest==northwest_before);
  bool contains_local_boundary=false;
  for (size_t i=1;i<connected.poses.size();++i) {
    const auto & a=connected.poses[i-1].pose.position;
    const auto & b=connected.poses[i].pose.position;
    require(std::hypot(b.x-a.x,b.y-a.y)<=.010000001);
    if (std::abs(std::hypot(b.x-measured.pose.position.x,b.y-measured.pose.position.y)-
      .05*std::sqrt(2.))<1e-10) contains_local_boundary=true;
  }
  require(contains_local_boundary);
  for (const auto & p : northwest.poses) {
    if (std::hypot(p.pose.position.x-measured.pose.position.x,
      p.pose.position.y-measured.pose.position.y)>.05*std::sqrt(2.)+1e-6) {
      require(std::find(connected.poses.begin(),connected.poses.end(),p)!=connected.poses.end());
    }
  }
  // If that partial-segment connector is blocked, preserve the checked corner.
  auto blocked_partial=[&](const auto & a,const auto & b) {
    return !(a==measured && b.pose.position.x<8. && b.pose.position.x>7.95);
  };
  auto conservative=bobac3_navigation::connectEndpointsAndDensify(
    northwest,measured,northwest_goal,.05,blocked_partial);
  require(std::any_of(conservative.poses.begin(),conservative.poses.end(),
    [&](const auto & p) { return p==northwest.poses[1]; }));
  std::cout << "PASS: exact start, all raw vertices and geometry preserved, <=1cm spacing; invalid connector rejected\n";
}
