// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <recon_controllers/joint_position_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace recon_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  
  sub_joint_ = node_handle.subscribe(
      "target_joint", 20, &JointPositionController::jointCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("JointPositionController: Could not read parameter arm_id");
    return false;
  }

  std::vector<double> k_p;
  if (!node_handle.getParam("p_gains", k_p)) {
    return false;
  }
  k_p_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(k_p.data());
  
  std::vector<double> k_d;
  if (!node_handle.getParam("d_gains", k_d)) {
    return false;
  }
  k_d_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(k_d.data());

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointPositionController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPositionController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPositionController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPositionController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  set_target_joint_action_server_ = std::make_unique<actionlib::SimpleActionServer<set_target_jointAction>>(
    node_handle, 
    "set_target_joint", 
    boost::bind(&JointPositionController::executeCB, this, _1), 
    false);

  set_target_joint_action_server_->start();

  return true;
}

void JointPositionController::starting(const ros::Time& /*time*/) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  q_d_ = q_initial;
  elapsed_time_ = 0;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  Eigen::VectorXd tau_d(7);

  if(!motion_finished_) {
    elapsed_time_ += period.toSec();
    double scaling = std::sin (M_PI / 4 * (1 - std::cos(M_PI / duration_ * elapsed_time_)));
    Eigen::Map<Eigen::Matrix<double, 7, 1>> delta_q(DELTA_Q_.data());
    if (elapsed_time_ <= duration_) {
      q_d_ << initial_joints_ + scaling * delta_q;
    } else {
      motion_finished_ = true;
    }
    
  }

  tau_d << k_p_.asDiagonal() * (q_d_ - q) - k_d_.asDiagonal() * dq;
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

void JointPositionController::stopping(const ros::Time&) {
  motion_finished_ = true;
}

Eigen::Matrix<double, 7, 1> JointPositionController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void JointPositionController::jointCallback(
  const recon_controllers::jointConstPtr& msg) {
    q_d_ << msg->joints[0], msg->joints[1], msg->joints[2], msg->joints[3], msg->joints[4], msg->joints[5], msg->joints[6];
  }
}  // namespace recon_controllers

PLUGINLIB_EXPORT_CLASS(recon_controllers::JointPositionController,
                       controller_interface::ControllerBase)
