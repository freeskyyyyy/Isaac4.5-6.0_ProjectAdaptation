#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav_msgs/msg/path.hpp"

namespace bobac3_navigation {
// Preserve every grid vertex. Add only the real start connector and collinear samples.
inline nav_msgs::msg::Path connectAndDensify(
  const nav_msgs::msg::Path & raw, const geometry_msgs::msg::PoseStamped & start,
  double resolution)
{
  if (raw.poses.empty()) return raw;
  if (start.header.frame_id != raw.header.frame_id ||
      !std::isfinite(resolution) || resolution <= 0.) {
    throw std::invalid_argument("invalid connector frame/resolution");
  }
  auto finite = [](const auto & p) {
    return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y);
  };
  if (!finite(start) || !std::all_of(raw.poses.begin(), raw.poses.end(), finite)) {
    throw std::invalid_argument("nonfinite path/connector");
  }
  const auto & first = raw.poses.front().pose.position;
  if (std::hypot(first.x-start.pose.position.x, first.y-start.pose.position.y) >
      resolution*std::sqrt(2.) + 1e-6) {
    throw std::invalid_argument("start connector exceeds one grid diagonal");
  }
  nav_msgs::msg::Path result; result.header=raw.header; result.poses.push_back(start);
  auto prev=start;
  for (const auto & end : raw.poses) {
    const auto & a=prev.pose.position; const auto & b=end.pose.position;
    const int steps=std::max(1, static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.01)));
    for (int i=1; i<steps; ++i) {
      auto p=end; double t=static_cast<double>(i)/steps;
      p.pose.position.x=a.x+t*(b.x-a.x); p.pose.position.y=a.y+t*(b.y-a.y);
      result.poses.push_back(p);
    }
    result.poses.push_back(end); prev=end;
  }
  return result;
}

// Correct only the endpoint grid quantization, after checking each new segment.
// Interior grid vertices and detours outside the start's immediate cell remain.
template<class ValidSegment>
inline nav_msgs::msg::Path connectEndpointsAndDensify(
  const nav_msgs::msg::Path & raw, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, double resolution, ValidSegment valid)
{
  if (raw.poses.empty()) return raw;
  auto finite=[](const auto & p) {
    return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y);
  };
  auto distance=[](const auto & a,const auto & b) {
    return std::hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y);
  };
  if (!std::isfinite(resolution) || resolution<=0. ||
      start.header.frame_id!=raw.header.frame_id || goal.header.frame_id!=raw.header.frame_id ||
      !finite(start) || !finite(goal) || !std::all_of(raw.poses.begin(),raw.poses.end(),finite)) {
    throw std::invalid_argument("invalid endpoint frame/geometry/resolution");
  }
  const double limit=resolution*std::sqrt(2.)+1e-6;
  if (distance(start,raw.poses.front())>limit || distance(goal,raw.poses.back())>limit) {
    throw std::invalid_argument("endpoint connector exceeds one grid diagonal");
  }
  if (!valid(start,raw.poses.front()) || !valid(raw.poses.back(),goal)) {
    throw std::invalid_argument("endpoint connector enters blocked/unknown cell");
  }
  size_t first=0;
  bool partial_segment=false;
  auto cut=start;
  for (size_t i=1;i<raw.poses.size();++i) {
    if (distance(start,raw.poses[i])>limit) {
      // The final point inside the local circle need not be a grid vertex.
      // Otherwise a valid forward segment just outside the circle leaves a
      // quantized backward/sideways corner for a short controller lookahead.
      const auto & a=raw.poses[i-1].pose.position;
      const auto & b=raw.poses[i].pose.position;
      const double dx=b.x-a.x,dy=b.y-a.y;
      const double ox=a.x-start.pose.position.x,oy=a.y-start.pose.position.y;
      const double length2=dx*dx+dy*dy,dot=ox*dx+oy*dy;
      const double radius=resolution*std::sqrt(2.);
      const double discriminant=dot*dot-length2*(ox*ox+oy*oy-radius*radius);
      if (length2>1e-24 && discriminant>=0.) {
        const double t=(-dot+std::sqrt(discriminant))/length2;
        if (t>=0. && t<=1.) {
          cut=raw.poses[i];
          cut.pose.position.x=a.x+t*dx; cut.pose.position.y=a.y+t*dy;
          if (valid(start,cut)) { first=i; partial_segment=true; }
        }
      }
      break;
    }
    if (valid(start,raw.poses[i])) first=i;
  }
  nav_msgs::msg::Path trimmed; trimmed.header=raw.header;
  trimmed.poses.assign(raw.poses.begin()+first,raw.poses.end());
  if (partial_segment) trimmed.poses.insert(trimmed.poses.begin(),cut);
  if (distance(trimmed.poses.back(),goal)<1e-12) trimmed.poses.back()=goal;
  else trimmed.poses.push_back(goal);
  return connectAndDensify(trimmed,start,resolution);
}
}  // namespace bobac3_navigation
