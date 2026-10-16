#include "bobac3_navigation/feedback_step.hpp"
#include "nav2_velocity_smoother/velocity_smoother.hpp"
#include "rclcpp/create_timer.hpp"
#include <array>

namespace bobac3_navigation {
class SimTimeVelocitySmoother : public nav2_velocity_smoother::VelocitySmoother {
protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {
    auto result=VelocitySmoother::on_configure(state);
    if (result!=nav2_util::CallbackReturn::SUCCESS) return result;
    if (!get_parameter("use_sim_time").as_bool() || open_loop_ ||
        !std::isfinite(smoothing_frequency_) || smoothing_frequency_<=0) {
      RCLCPP_ERROR(get_logger(),"World smoother requires use_sim_time=true, CLOSED_LOOP, positive frequency");
      return nav2_util::CallbackReturn::FAILURE;
    }
    nav2_util::declare_parameter_if_not_declared(shared_from_this(),"max_odom_age",rclcpp::ParameterValue(.15));
    max_age_=get_parameter("max_odom_age").as_double();
    if (!std::isfinite(max_age_) || max_age_<=0) return nav2_util::CallbackReturn::FAILURE;
    period_=1./smoothing_frequency_;
    return result;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    auto result=VelocitySmoother::on_activate(state);
    if (result!=nav2_util::CallbackReturn::SUCCESS) return result;
    timer_->cancel(); timer_.reset();
    // Upstream dynamic frequency updates recreate a wall timer. Keep this trial immutable while active.
    dyn_params_handler_.reset();
    dyn_params_handler_=add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult result; result.successful=false;
      result.reason="Deactivate/reconfigure the simulation-clock world smoother before changing parameters";
      return result;
    });
    previous_=now().seconds();
    timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(period_),[this] { tick(); });
    RCLCPP_INFO(get_logger(),"Simulation-clock CLOSED_LOOP smoother; dt cap %.3fs, odom age limit %.3fs",period_,max_age_);
    return result;
  }
private:
  void faultStop(const char * reason) {
    // Fault stops bypass the normal ramp and are never counted as normal braking.
    auto zero=std::make_unique<geometry_msgs::msg::TwistStamped>();
    zero->header.stamp=now(); smoothed_cmd_pub_->publish(std::move(zero));
    last_cmd_=geometry_msgs::msg::TwistStamped(); command_.reset(); stopped_=true;
    RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Stopping world smoother: %s",reason);
  }
  void tick() {
    const double current=now().seconds();
    const auto feedback=odom_smoother_->getTwistStamped();
    const double stamp=rclcpp::Time(feedback.header.stamp).seconds();
    const auto step=feedbackStep(current,previous_,stamp,period_,max_age_); previous_=current;
    if (!command_) return;
    if (step.stop) {
      faultStop("stale/future odom or non-forward simulation clock");
      return;
    }
    std::array<double,3> measured{feedback.twist.linear.x,feedback.twist.linear.y,feedback.twist.angular.z};
    for (size_t i=0;i<3;++i) {
      if (!std::isfinite(measured[i]) || measured[i]<min_velocities_[i]-1e-6 ||
          measured[i]>max_velocities_[i]+1e-6) {
        faultStop("non-finite or absolute overspeed odom"); return;
      }
    }
    const auto age=now()-last_command_time_;
    if (age.nanoseconds()<0) { faultStop("command timestamp ahead of simulation clock"); return; }
    if (age>velocity_timeout_) command_->twist=geometry_msgs::msg::Twist();
    std::array<double,3> target{command_->twist.linear.x,command_->twist.linear.y,command_->twist.angular.z};
    std::array<double,3> previous{last_cmd_.twist.linear.x,last_cmd_.twist.linear.y,last_cmd_.twist.angular.z};
    std::array<double,3> reference{},output{};
    for (size_t i=0;i<3;++i) {
      if (!std::isfinite(target[i]) || !std::isfinite(previous[i])) {
        faultStop("non-finite request or previous output"); return;
      }
      target[i]=std::clamp(target[i],min_velocities_[i],max_velocities_[i]);
      // Use measured feedback for normal tracking. A zero request (or reversal)
      // must first brake the previous command to zero; delayed nonzero odom must
      // not keep regenerating that command indefinitely. Odom faults are still
      // supervised above throughout this ordinary, rate-limited braking phase.
      if (target[i]==0. || target[i]*previous[i]<0.) {
        target[i]=0.; reference[i]=previous[i];
      } else {
        reference[i]=measured[i];
      }
    }
    // Upstream constraints divide by smoothing_frequency_; give them the real bounded sim dt.
    const double nominal=smoothing_frequency_; smoothing_frequency_=1./step.dt;
    double eta=1.;
    if (scale_velocities_) {
      for (size_t i=0;i<3;++i) {
        const auto candidate=findEtaConstraint(reference[i],target[i],max_accels_[i],max_decels_[i]);
        if (candidate>0.) eta=std::min(eta,candidate);
      }
    }
    for (size_t i=0;i<3;++i) {
      const auto proposed=applyConstraints(reference[i],target[i],max_accels_[i],max_decels_[i],eta);
      // Also bound consecutive commands, not only changes from measured odom.
      const auto progress=std::clamp(proposed,std::min(previous[i],target[i]),std::max(previous[i],target[i]));
      output[i]=std::clamp(applyConstraints(previous[i],progress,max_accels_[i],max_decels_[i],1.),
        min_velocities_[i],max_velocities_[i]);
    }
    smoothing_frequency_=nominal;
    auto cmd=std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd->header=command_->header; cmd->header.stamp=now();
    cmd->twist.linear.x=output[0]; cmd->twist.linear.y=output[1]; cmd->twist.angular.z=output[2];
    last_cmd_=*cmd; stopped_=cmd->twist==geometry_msgs::msg::Twist();
    smoothed_cmd_pub_->publish(std::move(cmd));
    if (stopped_ && age>velocity_timeout_) command_.reset();
  }
  double previous_{0.},period_{.05},max_age_{.15};
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<bobac3_navigation::SimTimeVelocitySmoother>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface()); executor.spin();
  rclcpp::shutdown(); return 0;
}
