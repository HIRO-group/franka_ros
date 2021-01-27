// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/force_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include "std_msgs/Float64.h"


#include <franka/robot_state.h>
#include "pseudo_inversion.h"
namespace franka_example_controllers {

bool ForceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  
  for (int i = 0; i < 7; i++)
  {
    prev_qd[0] = 0.0;
  }

  tau_pub_0 = node_handle.advertise<std_msgs::Float64>("tau_est_0", 1);
  tau_pub_1 = node_handle.advertise<std_msgs::Float64>("tau_est_1", 1);
  tau_pub_2 = node_handle.advertise<std_msgs::Float64>("tau_est_2", 1);
  tau_pub_3 = node_handle.advertise<std_msgs::Float64>("tau_est_3", 1);
  tau_pub_4 = node_handle.advertise<std_msgs::Float64>("tau_est_4", 1);
  tau_pub_5 = node_handle.advertise<std_msgs::Float64>("tau_est_5", 1);
  tau_pub_6 = node_handle.advertise<std_msgs::Float64>("tau_est_6", 1);
  
  
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceExampleController::desiredMassParamCallback, this, _1, _2));

  return true;
}

void ForceExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();
}

void ForceExampleController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_F_ext_hat_K.data());

  // Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_T_EE.data());
  // robot_state.


  // get coriolis and mass matrices
  std::array<double, 49> mass = model_handle_->getMass();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  std::array<double, 7> joint_accs;
  for (int i = 0; i < 7; i++)
  { 
    joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec());
    // joint_accs[i] = 0.0;

    // std::cout << "current joint acc at " << i << ": " << robot_state.dq[i] << std::endl;
    // std::cout << "prev joint acc at " << i << ": " << prev_qd[i] << std::endl;


  }

  std::array<double, 3>fake_grav{0, 0, -9.81 * 0.73};

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

  // Eigen::MatrixXd expected_torques = jacobian.transpose()  * wrench;
  
  // tau_pub_6.publish(expected_torques(6));
  Eigen::VectorXd tau_est(7);
  tau_est = (mass_matrix * ddq) + coriolis_matrix + gravity + tau_ext_initial_;
  for (int i = 0; i < 7; i++)
  { 
  }

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass_ * -9.81;
  // measure external torques
  // tau_ext_initial_ is a bias measurement
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  Eigen::MatrixXd expected_torques = (pinv  * (tau_measured - gravity)) - wrench;

  // std::cout << expected_torques << std::endl;
  tau_pub_0.publish(expected_torques(0));
  tau_pub_1.publish(expected_torques(1));
  tau_pub_2.publish(expected_torques(2));
  tau_pub_3.publish(expected_torques(3));
  tau_pub_4.publish(expected_torques(4));
  tau_pub_5.publish(expected_torques(5));
  // Publish tau error for plotting
  // tau_pub_0.publish(tau_est(0));
  // tau_pub_1.publish(tau_est(1));
  // tau_pub_2.publish(tau_est(2));
  // tau_pub_3.publish(tau_est(3));
  // tau_pub_4.publish(tau_est(4));
  // tau_pub_5.publish(tau_est(5));
  // tau_pub_6.publish(tau_est(6));

  
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  // tau_pub_0.publish(tau_error_(0));
  // tau_pub_1.publish(tau_error_(1));
  // tau_pub_2.publish(tau_error_(2));
  // tau_pub_3.publish(tau_error_(3));
  // tau_pub_4.publish(tau_error_(4));
  // tau_pub_5.publish(tau_error_(5));
  // tau_pub_6.publish(tau_error_(6));

  for (size_t i = 0; i < 7; ++i) {

    joint_handles_[i].setCommand(tau_cmd(i));
    prev_qd[i] = robot_state.dq[i];
  }

  // Update signals changed online through dynamic reconfigure
  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

void ForceExampleController::desiredMassParamCallback(
    franka_example_controllers::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  target_mass_ = config.desired_mass;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
}

Eigen::Matrix<double, 7, 1> ForceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForceExampleController,
                       controller_interface::ControllerBase)
