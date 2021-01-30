// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

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

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  for (int i = 0; i < 7; i++)
  {
    prev_qd[i] = 0.0;
  }

  ext_cart_force_pub_x = node_handle.advertise<std_msgs::Float64>("ext_cart_force_x", 1);
  ext_cart_force_pub_y = node_handle.advertise<std_msgs::Float64>("ext_cart_force_y", 1);
  ext_cart_force_pub_z = node_handle.advertise<std_msgs::Float64>("ext_cart_force_z", 1);


  x_dot_pub = node_handle.advertise<std_msgs::Float64>("x_dot", 1);
  y_dot_pub = node_handle.advertise<std_msgs::Float64>("y_dot", 1);
  z_dot_pub = node_handle.advertise<std_msgs::Float64>("z_dot", 1);



  cart_ext_pub1 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub1", 1);
  cart_ext_pub2 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub2", 1);
  cart_ext_pub3 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub3", 1);
  cart_ext_sum = node_handle.advertise<std_msgs::Float64>("cart_ext_sum", 1);
  // tau_pub_3 = node_handle.advertise<std_msgs::Float64>("tau_est_3", 1);
  // tau_pub_4 = node_handle.advertise<std_msgs::Float64>("tau_est_4", 1);
  // tau_pub_5 = node_handle.advertise<std_msgs::Float64>("tau_est_5", 1);
  // tau_pub_6 = node_handle.advertise<std_msgs::Float64>("tau_est_6", 1);
  this->signal_parser_x = zScore(node_handle, "x");
  this->signal_parser_y = zScore(node_handle, "y");
  this->signal_parser_z = zScore(node_handle, "z");


  this->loops_without_signal = 0;
  this->loops_with_signal = 0;

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "JointVelocityExampleController: Robot is not in the expected starting position for "
    //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
    //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
}



void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {


  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_filtered(robot_state.tau_ext_hat_filtered.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_dot(robot_state.dq.data());


  Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_F_ext_hat_K.data());


  // get cartesian velocities

  Eigen::MatrixXd x_dot = jacobian * q_dot;

  x_dot_pub.publish(x_dot(0));
  y_dot_pub.publish(x_dot(1));
  z_dot_pub.publish(x_dot(2));
  double t1 = signal_parser_x.updateThreshold(x_dot(0));
  double t2 = signal_parser_y.updateThreshold(x_dot(1));
  double t3 = signal_parser_z.updateThreshold(x_dot(2));

  cart_ext_pub1.publish(t1);
  cart_ext_pub2.publish(t2);
  cart_ext_pub3.publish(t3);


  std::array<double, 49> mass = model_handle_->getMass();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  std::array<double, 7> joint_accs;
  for (int i = 0; i < 7; i++)
  { 
    // joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec() * 10);
    joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec() * 10);
    // joint_accs[i] = 0.0;

    //std::cout << "current joint acc at " << i << ": " << joint_accs[i] << std::endl;
    // std::cout << "prev joint acc at " << i << ": " << prev_qd[i] << std::endl;
    prev_qd[i] = robot_state.dq[i];


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

  // Eigen::MatrixXd ext_cartesian_wrench = jacobian.transpose()  * wrench;
  
  // tau_pub_6.publish(ext_cartesian_wrench(6));
  Eigen::VectorXd tau_est(7);
  tau_est = (mass_matrix * ddq) + coriolis_matrix + gravity + tau_ext_initial_;
  // for (int i = 0; i < 7; i++)
  // { 
  // }

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = 0 * -9.81;
  // measure external torques
  // tau_ext_initial_ is a bias measurement
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  // Eigen::MatrixXd ext_cartesian_wrench = (pinv  * (tau_measured - gravity -  coriolis_matrix)) - wrench;
  // Eigen::MatrixXd ext_cartesian_wrench = (pinv  * (tau_measured - gravity -  coriolis_matrix - (mass_matrix * ddq))) - wrench;
  Eigen::MatrixXd ext_cartesian_wrench = (pinv  * (tau_measured - gravity -  coriolis_matrix - (0.1 * (mass_matrix * ddq)))) - wrench;

  // ext_cartesian_wrench = tau_filtered;

  elapsed_time_ += period;
  desired_force_torque(2) = desired_mass_ * -9.81;

 

  // std::tuple<bool, float> signal_ret_x = signal_parser_x.getSignal(ext_cartesian_wrench(0));
  // std::tuple<bool, float> signal_ret_y = signal_parser_y.getSignal(ext_cartesian_wrench(1));
  // std::tuple<bool, float> signal_ret_z = signal_parser_z.getSignal(ext_cartesian_wrench(2));
  // std::tuple<bool, float> signal_ret_sum =  signal_parser_sum.getSignal(std::abs(ext_cartesian_wrench(0)) + std::abs(ext_cartesian_wrench(1)) + std::abs(ext_cartesian_wrench(2)));
  // display external cartesian force and convert to velocities via cartesian compliance values

  // Eigen::Vector3d f = Eigen::Vector3d(std::get<1>(signal_ret_x), std::get<1>(signal_ret_y), std::get<1>(signal_ret_z));
  Eigen::Vector3d cartesian_velocities = Eigen::Vector3d(0.5 * ext_cartesian_wrench(0), 0.5 * ext_cartesian_wrench(1), 0.5 * ext_cartesian_wrench(2));
  
  // convert to joint space and get joint velocities
  Eigen::MatrixXd jacobian_pinv;
  jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::VectorXd q_d = jacobian_pinv * cartesian_velocities;

  // publish externally detected cartesian forces
  // cart_ext_pub1.publish(ext_cartesian_wrench(0));
  // cart_ext_pub2.publish(ext_cartesian_wrench(1));
  // cart_ext_pub3.publish(ext_cartesian_wrench(2));
  std::tuple<bool, float> x_signal_ret = signal_parser_x.getSignal(ext_cartesian_wrench(0));
  // cart_ext_sum.publish(double(std::get<1>(signal_ret_sum)));

  // double external
  // signal_pub.publish(double(std::get<1>(signal_ret_x)));
  // signal_pub.publish(double(std::get<1>(signal_ret_x)));
  // signal_pub.publish(double(std::get<1>(signal_ret_x)));

  // mean_pub.publish(signal_parser_sum.current_mean);
  // std_dev_positive_pub.publish(signal_parser_sum.current_mean + signal_parser_sum.current_stdDev * signal_parser_sum.threshold);
  // std_dev_negative_pub.publish(signal_parser_sum.current_mean - signal_parser_sum.current_stdDev * signal_parser_sum.threshold);



  // if (std::get<0>(signal_ret_sum)){
  //   this->loops_with_signal++;
  //   if (this->loops_with_signal > 15){
  //     signal_parser_sum.threshold = 15;
  //     this->loops_without_signal = 0;
  //     std::cout << "pause_movement:" << this->loops_with_signal << std::endl;
  //     this->pause_movement = true;
  //   }

  // }else{
  //   this->loops_with_signal = 0;
   
  //   // this->loops_without_signal++;
  //   // if(this->loops_without_signal > 1000){
  //   //   this->loops_with_signal = 0;
  //   //   std::cout << "normal_movement:" << this->loops_without_signal << std::endl;
  //   //   this->pause_movement = false;
  //   // }

  // }


  if (std::get<0>(x_signal_ret)){
    this->loops_with_signal++;
    if (this->loops_with_signal > 10){
      this->loops_without_signal = 0;
      std::cout << "pause_movement:" << this->loops_with_signal << std::endl;
      this->pause_movement = true;
    }

  }else{
    this->loops_with_signal = 0;
   
    // this->loops_without_signal++;
    // if(this->loops_without_signal > 1000){
    //   this->loops_with_signal = 0;
    //   std::cout << "normal_movement:" << this->loops_without_signal << std::endl;
    //   this->pause_movement = false;
    // }
  }
  
  ext_cart_force_pub_x.publish(ext_cartesian_wrench(0));
  ext_cart_force_pub_y.publish(ext_cartesian_wrench(1));
  ext_cart_force_pub_z.publish(ext_cartesian_wrench(2));
  // tau_pub_3.publish(ext_cartesian_wrench(3));
  // tau_pub_4.publish(ext_cartesian_wrench(4));
  // tau_pub_5.publish(ext_cartesian_wrench(5));

    ros::Duration time_max(4.0);
    double omega_max = 0.1;
    double cycle = std::floor(
    std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
    double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));
    omega *=3;


  if (this->pause_movement){    
    // Just slow down the robot after contact is made
    //omega *= 0.1;
    omega = 0;
  }

  for (auto joint_handle : velocity_joint_handles_) {
      joint_handle.setCommand(omega);
  }

}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
