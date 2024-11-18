// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hiro_joint_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool HIROJointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  /* CB function for curobo*/
  sub_command_ = node_handle.subscribe<std_msgs::Float32MultiArray>(
                  "/cu_joint_solution", 1, &HIROJointImpedanceExampleController::cuRoboCommandCb, this); 

  sub_impedance_change_bool_ = node_handle.subscribe<std_msgs::Bool>(
                  "/impedance_change_bool", 1, &HIROJointImpedanceExampleController::impedanceChangeBoolCb, this);
                  
  /* CB function for xbox_input file*/
  sub_command2_ = node_handle.subscribe<sensor_msgs::JointState>(
                  "/relaxed_ik/joint_angle_solutions", 1, &HIROJointImpedanceExampleController::xboxCommandCb, this); 

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceExampleController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  for (int i = 0; i < 7; i++){
            k_gains_[i] = non_zero_imp_k[i];
            d_gains_[i] = non_zero_imp_d[i];
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void HIROJointImpedanceExampleController::starting(const ros::Time& /*time*/) {
  pose_from_cb_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

void HIROJointImpedanceExampleController::xboxCommandCb(const sensor_msgs::JointState::ConstPtr& joint_pos_commands) {
    for (int i = 0; i < 7; i++) joint_positions_[i] = joint_pos_commands->position[i];
    this->callback_done_once = true;
}

void HIROJointImpedanceExampleController::impedanceChangeBoolCb(const std_msgs::Bool::ConstPtr& impedance_change_bool){
    std::lock_guard<std::mutex> q_and_qdot_lock_mutex(joint_position_and_velocity_d_target_mutex_);
    if(impedance_change_bool->data){
        for (int i = 0; i < 7; i++){
            k_gains_[i] = 0.0;
            d_gains_[i] = 0.0;
        }
        // pose_from_cb_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        // std::array<double, 16> pose_desired = pose_from_cb_;
        // // std::cout << "Curr pose, impedance zero: " << pose_desired[12] << " " << pose_desired[13] << " " << pose_desired[14] << std::endl;
        // cartesian_pose_handle_->setCommand(pose_desired);
        // this->callback_done_once = true;
    }
    else{
        // pose_from_cb_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        // std::array<double, 16> pose_desired = pose_from_cb_;
        // cartesian_pose_handle_->setCommand(pose_desired);
        // for (size_t i = 0; i < 7; ++i) {
        //     joint_positions_[i] = cartesian_pose_handle_->getRobotState().q[i];
        // }
        
        for (int i = 0; i < 7; i++){
            k_gains_[i] = non_zero_imp_k[i];
            d_gains_[i] = non_zero_imp_d[i];
        }

        // std::cout << "Curr pose, impedance a trillion: " << pose_desired[12] << " " << pose_desired[13] << " " << pose_desired[14] << std::endl;
    }
}

void HIROJointImpedanceExampleController::cuRoboCommandCb(const std_msgs::Float32MultiArray::ConstPtr& joint_pos_commands) {
    std::lock_guard<std::mutex> q_and_qdot_lock_mutex(joint_position_and_velocity_d_target_mutex_);
    for (int i = 0; i < 7; i++) joint_positions_[i] = joint_pos_commands->data[i];
    this->callback_done_once = true;
}

void HIROJointImpedanceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
  if (angle_ > 2 * M_PI) {
    angle_ -= 2 * M_PI;
  }

  std::array<double, 16> pose_desired = pose_from_cb_;
  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();

  cartesian_pose_handle_->setCommand(pose_desired);
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  if(this->callback_done_once){
    std::lock_guard<std::mutex> q_and_qdot_lock_mutex(joint_position_and_velocity_d_target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            k_gains_[i] * (joint_positions_[i] - robot_state.q[i]) +
                            d_gains_[i] * ((joint_positions_[i] - robot_state.q[i]) - dq_filtered_[i]);
    }
    joint_position_and_velocity_d_target_mutex_.unlock();
  }else{
    for (size_t i = 0; i < 7; ++i) {
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                            d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
    }
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }
  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }
  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> HIROJointImpedanceExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HIROJointImpedanceExampleController,
                       controller_interface::ControllerBase)
