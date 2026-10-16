#include <stdexcept>
#include <cmath>
#include "bobac3_navigation/feedback_step.hpp"
void require(bool value) { if (!value) throw std::runtime_error("navigation offline regression failed"); }
int main() {
  using bobac3_navigation::feedbackStep;
  auto normal=feedbackStep(1.05,1.,1.04,.05,.15);
  require(!normal.stop && std::abs(normal.dt-.05)<1e-9);
  auto fast=feedbackStep(1.033333,1.,1.02,.05,.15);
  require(!fast.stop && fast.dt<.034);
  auto delayed=feedbackStep(1.1,1.,1.09,.05,.15);
  require(!delayed.stop && delayed.dt==.05);
  require(feedbackStep(1.1,1.,.9,.05,.15).stop);
  require(feedbackStep(.5,1.,.49,.05,.15).stop);
  require(feedbackStep(1.,1.,.99,.05,.15).stop);
  require(feedbackStep(1.,.95,1.1,.05,.15).stop);
  require(feedbackStep(1.,.95,0.,.05,.15).stop);
  require(feedbackStep(NAN,.95,1.,.05,.15).stop);
}
