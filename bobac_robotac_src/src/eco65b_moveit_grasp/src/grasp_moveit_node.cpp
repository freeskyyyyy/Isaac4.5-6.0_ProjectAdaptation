#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <chrono>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <eco65b_grasp_interfaces/action/grasp_to_point.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

class GraspMoveItNode final : public rclcpp::Node {
public:
  using GraspToPoint = eco65b_grasp_interfaces::action::GraspToPoint;
  using GoalHandleGraspToPoint = rclcpp_action::ServerGoalHandle<GraspToPoint>;

  GraspMoveItNode()
      : Node("eco65b_grasp_moveit_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    action_name_ = this->declare_parameter<std::string>("action_name", "/grasp_to_point");
    grasp_topic_ = this->declare_parameter<std::string>("grasp_topic", "/grasp_point_base");
    use_topic_ = this->declare_parameter<bool>("use_topic", false);

    hand_command_topic_ = this->declare_parameter<std::string>("hand_command_topic", "/hand_command");
    move_group_name_ = this->declare_parameter<std::string>("move_group_name", "eco65_arm");
    ee_link_ = this->declare_parameter<std::string>("ee_link", "link_6");
    finger_link_ = this->declare_parameter<std::string>("finger_link", "gripper_l_link_finger");
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "base_link_arm");
    plan_only_ = this->declare_parameter<bool>("plan_only", false);
    //pre_grasp_offset_ = this->declare_parameter<double>("pre_grasp_offset", 0.05);
    //finger_tip_offset_ = this->declare_parameter<double>("finger_tip_offset", 0.03);

    // Fixed Z offset from ee_link to finger_link (measured from TF: 0.169m)
    use_fixed_ee_offset_ = this->declare_parameter<bool>("use_fixed_ee_offset", true);
    fixed_ee_offset_z_ = this->declare_parameter<double>("fixed_ee_offset_z", 0.166);

    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/arm/joint_states");

    arm_joints_ = this->declare_parameter<std::vector<std::string>>(
        "arm_joints", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    hand_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(hand_command_topic_, 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, 50,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->on_joint_state(*msg); });

    if (use_topic_) {
      grasp_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
          grasp_topic_, 10,
          [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { this->on_grasp_point(*msg); });
    }

    action_server_ = rclcpp_action::create_server<GraspToPoint>(
        this, action_name_,
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GraspToPoint::Goal> goal) {
          (void)uuid;
          if (active_goal_) {
            RCLCPP_WARN(get_logger(), "Busy: rejecting new goal");
            return rclcpp_action::GoalResponse::REJECT;
          }
          (void)goal;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandleGraspToPoint> goal_handle) {
          (void)goal_handle;
          if (move_group_) {
            move_group_->stop();
          }
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandleGraspToPoint> goal_handle) { this->handle_accepted(goal_handle); });

    RCLCPP_INFO(get_logger(), "action_name=%s use_topic=%d grasp_topic=%s", action_name_.c_str(), use_topic_ ? 1 : 0,
                grasp_topic_.c_str());
    RCLCPP_INFO(get_logger(), "move_group=%s ee_link=%s finger_link=%s planning_frame=%s plan_only=%d", move_group_name_.c_str(),
                ee_link_.c_str(), finger_link_.c_str(), planning_frame_.c_str(), plan_only_);
  }

  void init_moveit() {
    // MoveGroupInterface reads robot_description(_semantic) from *this node's* parameters.
    // Mirror the values from /move_group so the interface can construct the RobotModel.
    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    if (!client->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_FATAL(get_logger(), "Timed out waiting for /move_group parameter service");
      throw std::runtime_error("/move_group param service not available");
    }

    auto params = client->get_parameters({
        "robot_description",
        "robot_description_semantic",
        // Kinematics config for the planning group (so this node can solve IK)
        "robot_description_kinematics." + move_group_name_ + ".kinematics_solver",
        "robot_description_kinematics." + move_group_name_ + ".kinematics_solver_attempts",
        "robot_description_kinematics." + move_group_name_ + ".kinematics_solver_search_resolution",
        "robot_description_kinematics." + move_group_name_ + ".kinematics_solver_timeout",
    });
    if (params.size() < 2u) {
      RCLCPP_FATAL(get_logger(), "Expected at least 2 parameters from /move_group, got %zu", params.size());
      throw std::runtime_error("Missing robot_description parameters");
    }
    for (const auto &p : params) {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        continue;
      }
      // Parameters from /move_group have mixed types (URDF/SRDF strings, kinematics ints/doubles, etc.).
      // Mirror them onto this node so MoveGroupInterface can load the full configuration.
      if (!this->has_parameter(p.get_name())) {
        this->declare_parameter(p.get_name(), p.get_parameter_value());
      }
      this->set_parameter(p);
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), move_group_name_);
    move_group_->setPoseReferenceFrame(planning_frame_);
    move_group_->setEndEffectorLink(ee_link_);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized");

    // Warm up internal state monitors / RobotModel access early and fail fast if something is wrong.
    // This avoids a hard crash later when a grasp message arrives.
    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      RCLCPP_FATAL(get_logger(), "MoveIt RobotModel unavailable after MoveGroupInterface init");
      throw std::runtime_error("MoveGroupInterface RobotModel unavailable");
    }
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState &msg) {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (i < msg.position.size()) {
        latest_joint_pos_[msg.name[i]] = msg.position[i];
      }
    }
  }

  std::optional<std::vector<double>> current_arm_joint_positions() const {
    std::vector<double> pos;
    pos.reserve(arm_joints_.size());
    for (const auto &j : arm_joints_) {
      auto it = latest_joint_pos_.find(j);
      if (it == latest_joint_pos_.end()) {
        return std::nullopt;
      }
      pos.push_back(it->second);
    }
    return pos;
  }

  void on_grasp_point(const geometry_msgs::msg::PointStamped &p) {
    if (active_goal_) {
      // Busy: ignore topic targets to avoid replanning thrash.
      return;
    }

    geometry_msgs::msg::PoseStamped commanded;
    int32_t error_code = 0;
    std::string message;
    (void)execute_once(p, std::nullopt, "", "", /*plan_only_override=*/false, commanded, error_code, message, nullptr, nullptr);
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGraspToPoint> goal_handle) {
    if (active_goal_) {
      auto result = std::make_shared<GraspToPoint::Result>();
      result->success = false;
      result->error_code = -10;
      result->message = "Busy";
      goal_handle->abort(result);
      return;
    }

    active_goal_ = goal_handle;
    std::thread([this, goal_handle]() {
      const auto goal = goal_handle->get_goal();

      auto publish_feedback = [&](const std::string &state) {
        auto fb = std::make_shared<GraspToPoint::Feedback>();
        fb->state = state;
        goal_handle->publish_feedback(fb);
      };

      geometry_msgs::msg::PointStamped p = goal->target;
      if (p.header.frame_id.empty()) {
        p.header.frame_id = planning_frame_;
      }

      if (p.header.frame_id != planning_frame_) {
        try {
          const auto tf = tf_buffer_.lookupTransform(planning_frame_, p.header.frame_id, tf2::TimePointZero);
          geometry_msgs::msg::PointStamped p2;
          tf2::doTransform(p, p2, tf);
          p = p2;
        } catch (const tf2::TransformException &e) {
          auto result = std::make_shared<GraspToPoint::Result>();
          result->success = false;
          result->error_code = -1;
          result->message = std::string("TF failed: ") + e.what();
          goal_handle->abort(result);
          active_goal_.reset();
          return;
        }
      }

      geometry_msgs::msg::Vector3Stamped n = goal->normal;
      geometry_msgs::msg::Vector3Stamped sa = goal->short_axis;
      if (n.header.frame_id.empty()) {
        n.header.frame_id = planning_frame_;
      }
      if (sa.header.frame_id.empty()) {
        sa.header.frame_id = planning_frame_;
      }

      if (n.header.frame_id != planning_frame_) {
        try {
          const auto tf = tf_buffer_.lookupTransform(planning_frame_, n.header.frame_id, tf2::TimePointZero);
          geometry_msgs::msg::Vector3Stamped n2;
          tf2::doTransform(n, n2, tf);
          n = n2;
        } catch (const tf2::TransformException &) {
        }
      }
      if (sa.header.frame_id != planning_frame_) {
        try {
          const auto tf = tf_buffer_.lookupTransform(planning_frame_, sa.header.frame_id, tf2::TimePointZero);
          geometry_msgs::msg::Vector3Stamped sa2;
          tf2::doTransform(sa, sa2, tf);
          sa = sa2;
        } catch (const tf2::TransformException &) {
        }
      }

      const std::string ee = goal->ee_link.empty() ? ee_link_ : goal->ee_link;
      const std::string finger = goal->finger_link.empty() ? finger_link_ : goal->finger_link;
      const bool plan_only = goal->plan_only ? true : plan_only_;

      publish_feedback("PLANNING");

      geometry_msgs::msg::PoseStamped commanded;
      int32_t error_code = 0;
      std::string message;
      const bool ok = execute_once(p, goal_handle, ee, finger, plan_only, commanded, error_code, message, &n, &sa);

      auto result = std::make_shared<GraspToPoint::Result>();
      result->success = ok;
      result->error_code = error_code;
      result->message = message;
      result->commanded_pose = commanded;

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else if (ok) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }

      active_goal_.reset();
    }).detach();
  }

  bool execute_once(const geometry_msgs::msg::PointStamped &p,
                    const std::optional<std::shared_ptr<GoalHandleGraspToPoint>> &goal_handle_opt,
                    const std::string &ee_link_override,
                    const std::string &finger_link_override,
                    bool plan_only_override,
                    geometry_msgs::msg::PoseStamped &commanded_out,
                    int32_t &error_code_out,
                    std::string &message_out,
                    const geometry_msgs::msg::Vector3Stamped *normal_opt,
                    const geometry_msgs::msg::Vector3Stamped *short_axis_opt) {
    if (!move_group_) {
      message_out = "MoveGroupInterface not initialized";
      error_code_out = -2;
      return false;
    }

    const std::string ee = ee_link_override.empty() ? ee_link_ : ee_link_override;
    const std::string finger = finger_link_override.empty() ? finger_link_ : finger_link_override;
    const bool plan_only = plan_only_override ? true : plan_only_;

    auto publish_feedback = [&](const std::string &state) {
      if (goal_handle_opt && *goal_handle_opt) {
        auto fb = std::make_shared<GraspToPoint::Feedback>();
        fb->state = state;
        (*goal_handle_opt)->publish_feedback(fb);
      }
    };
    (void)publish_feedback;

    if (goal_handle_opt && *goal_handle_opt && (*goal_handle_opt)->is_canceling()) {
      message_out = "Canceled";
      error_code_out = 1;
      return false;
    }

    auto maybe_pos = current_arm_joint_positions();
    if (!maybe_pos) {
      message_out = "Missing arm joint states";
      error_code_out = -3;
      return false;
    }

    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      message_out = "MoveIt RobotModel unavailable";
      error_code_out = -4;
      return false;
    }

    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();

    const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(move_group_name_);
    if (!jmg) {
      message_out = "JointModelGroup not found";
      error_code_out = -5;
      return false;
    }

    state.setJointGroupPositions(jmg, *maybe_pos);
    state.update();

    const Eigen::Isometry3d &ee_tf = state.getGlobalLinkTransform(ee);
    Eigen::Quaterniond ee_q(ee_tf.rotation());

    tf2::Vector3 t_ee_to_finger(0.0, 0.0, 0.0);

    if (use_fixed_ee_offset_) {
      // Use fixed Z offset only to avoid dynamic TF lookup issues when gripper opens/closes
      // X and Y offsets are handled by orientation, only Z matters for position
      t_ee_to_finger = tf2::Vector3(0.0, 0.0, fixed_ee_offset_z_);
      RCLCPP_INFO_ONCE(get_logger(), "Using fixed ee_to_finger Z offset: %.3f m", fixed_ee_offset_z_);
    } else {
      // Dynamic TF lookup (original behavior)
      try {
        const auto tf_ee_to_finger_msg = tf_buffer_.lookupTransform(ee, finger, tf2::TimePointZero);
        t_ee_to_finger = tf2::Vector3(tf_ee_to_finger_msg.transform.translation.x,
                                     tf_ee_to_finger_msg.transform.translation.y,
                                     tf_ee_to_finger_msg.transform.translation.z);
      } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed (%s -> %s): %s. Using zero offset.",
                             ee.c_str(), finger.c_str(), e.what());
      }
    }

    tf2::Quaternion q_ee(ee_q.x(), ee_q.y(), ee_q.z(), ee_q.w());
    tf2::Matrix3x3 R_ee(q_ee);
    const tf2::Vector3 t_in_planning = R_ee * t_ee_to_finger;

    geometry_msgs::msg::PoseStamped target;
    target.header = p.header;
    target.header.frame_id = planning_frame_;
    target.pose.position.x = p.point.x - t_in_planning.x();
    target.pose.position.y = p.point.y - t_in_planning.y();
    target.pose.position.z = p.point.z - t_in_planning.z();

    // Orientation: For optical-frame end-effector (Z=approach, X=gripper-width, Y=gripper-bottom):
    // - Z_link6 should point down (approach direction): Z = normal (not -normal)
    // - X_link6 should align with pen long-axis (gripper grasps along X)
    // - Y_link6 is computed via cross product
    bool have_orient = false;
    Eigen::Quaterniond q_choice = ee_q;
    if (normal_opt && short_axis_opt) {
      Eigen::Vector3d n(normal_opt->vector.x, normal_opt->vector.y, normal_opt->vector.z);
      // For yoloe_grasp we repurpose short_axis_opt to carry the mask long-axis direction.
      Eigen::Vector3d long_axis(short_axis_opt->vector.x, short_axis_opt->vector.y, short_axis_opt->vector.z);
      if (n.norm() > 1e-6 && long_axis.norm() > 1e-6) {
        Eigen::Vector3d z = n.normalized();

        RCLCPP_INFO(get_logger(), "Input normal: (%.3f, %.3f, %.3f)", n.x(), n.y(), n.z());
        RCLCPP_INFO(get_logger(), "Computed z-axis: (%.3f, %.3f, %.3f)", z.x(), z.y(), z.z());

        auto build_q_from_axis = [&](const Eigen::Vector3d &axis) -> std::optional<Eigen::Quaterniond> {
          // We want Y to align with axis (pen long-axis), projected onto plane orthogonal to z.
          Eigen::Vector3d y = axis - axis.dot(z) * z;
          if (y.norm() < 1e-6) {
            // axis is parallel to z, choose arbitrary y in the plane
            Eigen::Vector3d ref(0.0, 1.0, 0.0);
            if (std::abs(ref.dot(z)) > 0.9) {
              ref = Eigen::Vector3d(1.0, 0.0, 0.0);
            }
            y = ref - ref.dot(z) * z;
          }
          y.normalize();

          Eigen::Vector3d x = y.cross(z);
          if (x.norm() < 1e-6) {
            return std::nullopt;
          }
          x.normalize();
          y = z.cross(x);  // Re-orthogonalize
          x.normalize();

          // Build rotation matrix: columns are the axes of link_6 expressed in base frame
          Eigen::Matrix3d R;
          R.col(0) = x;
          R.col(1) = y;
          R.col(2) = z;

          Eigen::Quaterniond q(R);
          q.normalize();
          return q;
        };

        // Choose between long_axis and -long_axis by IK distance to current joints.
        // Also check IK feasibility and joint limits.
        moveit::core::RobotState ik_try(robot_model);
        ik_try.setToDefaultValues();
        ik_try.setJointGroupPositions(jmg, *maybe_pos);
        ik_try.update();
        std::vector<double> cur_j;
        ik_try.copyJointGroupPositions(jmg, cur_j);

        double best_cost = std::numeric_limits<double>::infinity();
        std::optional<Eigen::Quaterniond> best_q;

        // Try both directions of the long axis (180 degree symmetry for pen grasping)
        for (const auto &axis : {long_axis.normalized(), (-long_axis).normalized()}) {
          const auto q_opt = build_q_from_axis(axis);
          if (!q_opt) {
            continue;
          }

          const Eigen::Quaterniond &q = *q_opt;

          // CRITICAL: Verify that the Z-axis of this orientation points in the correct direction
          // For top-down grasping, Z should point down (same direction as normal)
          Eigen::Vector3d z_axis = q.toRotationMatrix().col(2);  // Extract Z column
          double z_alignment = z_axis.dot(z);  // Should be close to 1.0 if aligned
          if (z_alignment < 0.5) {
            // Z-axis is pointing in the wrong direction, skip this candidate
            RCLCPP_DEBUG(get_logger(), "Skipping candidate: Z-axis misaligned (dot=%.3f)", z_alignment);
            continue;
          }

          Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
          goal.translation() = Eigen::Vector3d(target.pose.position.x, target.pose.position.y, target.pose.position.z);
          goal.linear() = q.toRotationMatrix();

          moveit::core::RobotState ik_state_tmp(robot_model);
          ik_state_tmp.setToDefaultValues();
          ik_state_tmp.setJointGroupPositions(jmg, *maybe_pos);
          ik_state_tmp.update();
          if (!ik_state_tmp.setFromIK(jmg, goal, ee)) {
            continue;
          }

          // Check if solution satisfies joint limits
          if (!ik_state_tmp.satisfiesBounds(jmg)) {
            continue;
          }

          std::vector<double> sol;
          ik_state_tmp.copyJointGroupPositions(jmg, sol);

          // Cost function: weighted sum of joint distance and manipulability
          double joint_cost = 0.0;
          for (size_t i = 0; i < sol.size() && i < cur_j.size(); ++i) {
            double diff = sol[i] - cur_j[i];

            // For revolute joints (especially joint 6), consider the shortest angular path
            // Assume joints with large range (> 4 radians) are continuous revolute joints
            const auto* joint_model = jmg->getJointModels()[i];
            if (joint_model && joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
              const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
              if (revolute && revolute->isContinuous()) {
                // Normalize to [-π, π]
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;
              }
            }

            // Weight joint 6 more heavily to discourage unnecessary rotation
            double weight = (i == sol.size() - 1) ? 2.0 : 1.0;
            joint_cost += weight * std::abs(diff);
          }

          // Prefer solutions with better manipulability (further from singularities)
          double manipulability = 1.0;
          try {
            Eigen::MatrixXd jacobian = ik_state_tmp.getJacobian(jmg);
            if (jacobian.rows() > 0 && jacobian.cols() > 0) {
              Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
              auto singular_values = svd.singularValues();
              if (singular_values.size() > 0) {
                double min_sv = singular_values.minCoeff();
                manipulability = std::max(0.01, min_sv);
              }
            }
          } catch (...) {
            // If manipulability calculation fails, use default
          }

          // Combined cost: prioritize joint distance, but penalize poor manipulability
          double cost = joint_cost + 2.0 / manipulability;

          if (cost < best_cost) {
            best_cost = cost;
            best_q = q;
          }
        }

        if (best_q) {
          q_choice = *best_q;
          have_orient = true;

          // Log the chosen orientation for debugging
          Eigen::Vector3d euler = q_choice.toRotationMatrix().eulerAngles(0, 1, 2);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Grasp orientation: roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg",
                               euler[0] * 180.0 / M_PI, euler[1] * 180.0 / M_PI, euler[2] * 180.0 / M_PI);
        }
      }
    }

    if (have_orient) {
      target.pose.orientation.x = q_choice.x();
      target.pose.orientation.y = q_choice.y();
      target.pose.orientation.z = q_choice.z();
      target.pose.orientation.w = q_choice.w();

      // Debug: print the Z-axis of the chosen orientation
      Eigen::Vector3d z_axis = q_choice.toRotationMatrix().col(2);
      RCLCPP_INFO(get_logger(), "Final Z-axis in base frame: (%.3f, %.3f, %.3f)", z_axis.x(), z_axis.y(), z_axis.z());

      // 偏移计算已移到状态机中，这里不再应用偏移
      RCLCPP_INFO(get_logger(), "Target position (no offset applied): (%.3f, %.3f, %.3f)",
                  target.pose.position.x, target.pose.position.y, target.pose.position.z);
    } else {
      target.pose.orientation.x = ee_q.x();
      target.pose.orientation.y = ee_q.y();
      target.pose.orientation.z = ee_q.z();
      target.pose.orientation.w = ee_q.w();
    }

    commanded_out = target;

    moveit::core::RobotState ik_state(robot_model);
    ik_state.setToDefaultValues();
    ik_state.setJointGroupPositions(jmg, *maybe_pos);
    ik_state.update();

    const Eigen::Quaterniond q_target(target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y,
                                     target.pose.orientation.z);
    Eigen::Isometry3d target_eig = Eigen::Isometry3d::Identity();
    target_eig.translation() = Eigen::Vector3d(target.pose.position.x, target.pose.position.y, target.pose.position.z);
    target_eig.linear() = q_target.toRotationMatrix();

    if (!ik_state.setFromIK(jmg, target_eig, ee)) {
      message_out = "NO_IK";
      error_code_out = 2;
      return false;
    }

    // Normalize continuous revolute joints to shortest path
    std::vector<double> ik_solution;
    ik_state.copyJointGroupPositions(jmg, ik_solution);
    for (size_t i = 0; i < ik_solution.size() && i < maybe_pos->size(); ++i) {
      const auto* joint_model = jmg->getJointModels()[i];
      if (joint_model && joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
        const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
        if (revolute) {
          bool should_normalize = revolute->isContinuous();

          // Also normalize revolute joints with large range (> 2*PI)
          if (!should_normalize) {
            const auto& bounds = revolute->getVariableBounds()[0];
            double range = bounds.max_position_ - bounds.min_position_;
            if (range > 2.0 * M_PI) {
              should_normalize = true;
            }
          }

          if (should_normalize) {
            // Normalize to shortest path from current position
            double diff = ik_solution[i] - (*maybe_pos)[i];
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            ik_solution[i] = (*maybe_pos)[i] + diff;
            RCLCPP_INFO(get_logger(), "Normalized joint %zu (%s): current=%.3f, ik=%.3f -> %.3f (diff: %.3f)",
                        i, joint_model->getName().c_str(), (*maybe_pos)[i], ik_solution[i] - diff, ik_solution[i], diff);
          }
        }
      }
    }
    ik_state.setJointGroupPositions(jmg, ik_solution);
    ik_state.update();

    move_group_->setStartState(state);
    move_group_->setPoseTarget(target, ee);

    // Set slower speed for safer, more controlled motion
    move_group_->setMaxVelocityScalingFactor(0.2);  // 20% of max velocity
    move_group_->setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration

    // Use Cartesian path planning for more predictable, straight-line motion
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm resolution for Cartesian path

    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, trajectory);

    RCLCPP_INFO(get_logger(), "Cartesian path: %.2f%% achieved", fraction * 100.0);

    if (fraction < 0.95) {
      // If Cartesian path fails, fall back to joint-space planning
      RCLCPP_WARN(get_logger(), "Cartesian path only %.2f%% complete, falling back to joint-space planning", fraction * 100.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto rc = move_group_->plan(plan);
      if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
        message_out = "Planning failed";
        error_code_out = rc.val;
        return false;
      }
      trajectory = plan.trajectory;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = trajectory;

    // Always publish the planned joint trajectory to IsaacSim via /hand_command.
    // IsaacSim subscriber expects JointState positions.
    if (!plan.trajectory.joint_trajectory.joint_names.empty() && !plan.trajectory.joint_trajectory.points.empty()) {
      const auto &jt = plan.trajectory.joint_trajectory;
      // Build an index map from planned joint order to arm_joints_ order.
      std::vector<int> idx;
      idx.reserve(arm_joints_.size());
      for (const auto &name : arm_joints_) {
        int found = -1;
        for (size_t i = 0; i < jt.joint_names.size(); ++i) {
          if (jt.joint_names[i] == name) {
            found = static_cast<int>(i);
            break;
          }
        }
        idx.push_back(found);
      }

      // Trajectory speed scaling: 1.0 = original timing, 2.0 = 2x faster.
      double speed_scale = 1.0;
      if (!this->has_parameter("hand_speed_scale")) {
        speed_scale = this->declare_parameter<double>("hand_speed_scale", 1.0);
      } else {
        speed_scale = this->get_parameter("hand_speed_scale").as_double();
      }
      speed_scale = std::max(0.01, speed_scale);

      auto start = this->now();
      for (size_t pi = 0; pi < jt.points.size(); ++pi) {
        if (goal_handle_opt && *goal_handle_opt && (*goal_handle_opt)->is_canceling()) {
          message_out = "Canceled";
          error_code_out = 1;
          return false;
        }

        const auto &pt = jt.points[pi];
        if (pt.positions.size() != jt.joint_names.size()) {
          continue;
        }

        const rclcpp::Duration wait_dur = rclcpp::Duration(pt.time_from_start) * (1.0 / speed_scale);
        const auto target_time = start + wait_dur;
        while (this->now() < target_time) {
          if (goal_handle_opt && *goal_handle_opt && (*goal_handle_opt)->is_canceling()) {
            message_out = "Canceled";
            error_code_out = 1;
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->now();
        cmd.name = arm_joints_;
        cmd.position.resize(arm_joints_.size(), 0.0);
        for (size_t j = 0; j < arm_joints_.size(); ++j) {
          const int si = idx[j];
          if (si >= 0) {
            cmd.position[j] = pt.positions[static_cast<size_t>(si)];
          }
        }
        hand_cmd_pub_->publish(cmd);

        if (goal_handle_opt && *goal_handle_opt) {
          auto fb = std::make_shared<GraspToPoint::Feedback>();
          fb->state = "EXECUTING";
          (*goal_handle_opt)->publish_feedback(fb);
        }
      }
    }

    if (plan_only) {
      message_out = "Planned (plan_only)";
      error_code_out = 0;
      return true;
    }

    message_out = "Succeeded";
    error_code_out = 0;
    return true;
  }

  std::string action_name_;
  bool use_topic_;
  std::string grasp_topic_;
  std::string hand_command_topic_;
  std::string move_group_name_;
  std::string ee_link_;
  std::string planning_frame_;
  std::string joint_state_topic_;
  std::string finger_link_;
  bool plan_only_;
  //double pre_grasp_offset_;
  //double finger_tip_offset_;

  bool use_fixed_ee_offset_;
  double fixed_ee_offset_z_;

  std::vector<std::string> arm_joints_;
  std::unordered_map<std::string, double> latest_joint_pos_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr grasp_sub_;

  rclcpp_action::Server<GraspToPoint>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleGraspToPoint> active_goal_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GraspMoveItNode>();
  node->init_moveit();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
