// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <recon_controllers/joint.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <actionlib/server/simple_action_server.h>
#include <recon_controllers/set_target_jointAction.h>

namespace recon_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void stopping(const ros::Time&) override;
    

    void executeCB(const recon_controllers::set_target_jointGoalConstPtr &goal) {
        motion_finished_ = false;
        duration_ = goal->duration;
        std::array<double, 7> target_joints;
        std::copy(std::begin(goal->joints), std::end(goal->joints), std::begin(target_joints));
        std::array<double, 7> initial_joints = state_handle_->getRobotState().q;
        initial_joints_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_joints.data());
        std::transform(initial_joints.begin(), initial_joints.end(), 
                        target_joints.begin(), 
                        DELTA_Q_.begin(), 
                        [](double init_pose, double goal_pose)->double{
                            return goal_pose - init_pose;
                        });
        elapsed_time_ = 0;
        ros::Rate r(1);
        while(!motion_finished_) {
            r.sleep();
        }
        recon_controllers::set_target_jointResult result;
        std::array<double, 7> current_joint = state_handle_->getRobotState().q;
        bool success = std::equal(begin(current_joint), end(current_joint), 
                                    begin(target_joints), end(target_joints),
                                [](double current, double goal){
                                    return std::fabs(current - goal) < 0.005;
                                });
      
        result.success = success;
        set_target_joint_action_server_->setSucceeded(result);
        q_d_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(target_joints.data());
        return;
  };

 private:
 
  // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    Eigen::Matrix<double, 7, 1> k_p_;
    Eigen::Matrix<double, 7, 1> k_d_; 
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 7, 1> q_d_;
                                
    ros::Subscriber sub_joint_;
    void jointCallback(const recon_controllers::jointConstPtr& msg);
    bool motion_finished_ {true};
    // actionlib::SimpleActionServer<set_target_jointAction> set_target_joint_action_server_;
    std::unique_ptr<actionlib::SimpleActionServer<set_target_jointAction>> set_target_joint_action_server_;
    Eigen::Matrix<double, 7, 1> initial_joints_ {};
    ros::NodeHandle nh_;
    std::array<double, 7> DELTA_Q_;
    double elapsed_time_ {};
    double duration_ {};
};

}  // namespace recon_controllers
