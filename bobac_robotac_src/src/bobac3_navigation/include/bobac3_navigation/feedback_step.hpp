#pragma once
#include <algorithm>
#include <cmath>
namespace bobac3_navigation {
struct FeedbackStep { double dt; bool stop; };
inline FeedbackStep feedbackStep(double now, double previous, double odom, double period, double max_age) {
  if (!std::isfinite(now) || !std::isfinite(previous) || !std::isfinite(odom) ||
      !std::isfinite(period) || !std::isfinite(max_age) || period<=0 || max_age<=0 ||
      now<=previous || odom<=0 || now-odom>max_age || odom-now>.05) return {0.,true};
  // Delayed callbacks must not create a larger ramp step; early callbacks use actual dt.
  return {std::min(now-previous,period),false};
}
}
