// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hiro_joint_velocity_effort_controller.h>


#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include "std_msgs/Float64.h"


#include <franka/robot_state.h>
#include "pseudo_inversion.h"
#include <ros/ros.h>

#include <numeric>

namespace franka_example_controllers {

bool HIROVelocityEffortController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  

  for (int i = 0; i < 7; i++)
  {
    prev_qd[i] = 0.0;
    cum_sum(i) = 0.0;
    r(i) = 0;
  }

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "HIROVelocityEffortController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("HIROVelocityEffortController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("HIROVelocityEffortController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HIROVelocityEffortController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("HIROVelocityEffortController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }



// Added to allow for effort interface HIRO
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HIROVelocityEffortController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "HIROVelocityEffortController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HIROVelocityEffortController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HIROVelocityEffortController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  torques_publisher_.init(node_handle, "torque_comparison", 1);

  // stuff for our control work
  sub_command_ = node_handle.subscribe<std_msgs::Float64MultiArray>("command", 10, &HIROVelocityEffortController::jointCommandCb, this);

  commanded_joint_velocities.resize(7);
  for (int i = 0; i < 7; i++) {
    commanded_joint_velocities[i] = 0.0;
    pubs.push_back(node_handle.advertise<std_msgs::Float64>("command" + std::to_string(i), 1));
    pubs_fake.push_back(node_handle.advertise<std_msgs::Float64>("command_fake" + std::to_string(i), 1));

  }

  return true;
}

void HIROVelocityEffortController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
}

void HIROVelocityEffortController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());


  Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_F_ext_hat_K.data());


  std::array<double, 49> mass = model_handle_->getMass();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> joint_accs;

  for (int i = 0; i < 7; i++)
  { 
    // joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec() * 10);
    joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec());
    // joint_accs[i] = 0.0;

    //std::cout << "current joint acc at " << i << ": " << joint_accs[i] << std::endl;
    // std::cout << "prev joint acc at " << i << ": " << prev_qd[i] << std::endl;
    prev_qd[i] = robot_state.dq[i];


  }

  Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq(joint_accs.data());


  // NOTE: it's always good to convert to eigen!
  // convert into eigen to simplify math
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis_matrix(coriolis.data());
  // invert jac transpose
  Eigen::MatrixXd jacobian_transpose_pinv;
  // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  jacobian_transpose_pinv = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> pinv(jacobian_transpose_pinv.data());


  

  ros::Duration time_max(8.0);
  double omega_max = 0.1;
  double cycle = std::floor(
      std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  for (int i = 0; i < 7; i++) {
    // get joint velocity
    double qDot = robot_state.dq[i];
    double difference = omega - qDot;
    double p_error;
    p_error = p_gains[i] * difference;
    i_errors[i] += (difference * period.toSec());  
    double i_error = 1 * i_errors[i];
    double d_error = d_gains[i] * difference;
    double control_command = p_error + i_error + d_error;
    // set the torque command
    if (commanded_joint_velocities[i] > 0.01) {
      std::cout << control_command << std::endl;
    }
    t_c(i) = control_command;
    effort_joint_handles_[i].setCommand(control_command);
    

    pubs[i].publish(control_command);
    // pubs_fake[i].publish(r(i) - (control_command));
  }
  r =  (pinv * (tau_measured - (tau_J_d + gravity))) - wrench;
  for (int i =0; i<7; i++)
  {
    std::cout << r(i) << std::endl;
    pubs_fake[i].publish(r(i));

  }
  // r = (tau_measured - gravity - coriolis_matrix - (mass_matrix * dq) - tau_ext_initial_);

}

void HIROVelocityEffortController::jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_velocity_commands) {
  if (joint_velocity_commands->data.size() != 7) {
        ROS_ERROR_STREAM("HIROJointVelocityEffortController: Wrong number of joint position commands, got "
                        << joint_velocity_commands->data.size() << " instead of 7 commands!");
    }
    std::cout << "Received message!" << std::endl;

    for (int i = 0; i < 7; i++) {
      std::cout << "Joint velocity desired of " << i << ": " << joint_velocity_commands->data[i] <<  std::endl;
        // set the commanded joint velocities to be used in pid control
        commanded_joint_velocities[i] = joint_velocity_commands->data[i];
    }
}

void HIROVelocityEffortController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HIROVelocityEffortController,
                       controller_interface::ControllerBase)
