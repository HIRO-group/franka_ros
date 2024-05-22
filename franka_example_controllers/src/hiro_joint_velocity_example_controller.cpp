// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hiro_joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_msgs/FrankaState.h>

#include <iostream>
#include <thread>

#include <franka/exception.h>
// #include <franka/vacuum_gripper.h>

namespace franka_example_controllers {

using franka_gripper::homing;
using franka_gripper::HomingAction;
using franka_gripper::HomingGoalConstPtr;
using franka_gripper::HomingResult;

// using franka_gripper::move;
// using franka_gripper::MoveAction;
// using franka_gripper::MoveGoalConstPtr;
// using franka_gripper::MoveResult;
// using franka::VacuumGripper;
// using franka::VacuumGripperState;

bool HIROJointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  // Check if I can create the vac here. 
  // franka::VacuumGripper vacuum_gripper("192.168.0.198");
  // franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
  // std::cout << "***********Initial vacuum gripper state: " << vacuum_gripper_state << std::endl;
  // std::cout << "Aftr the init state has been got!" << std::endl;

  // if (!vacuum_gripper.vacuum(100, std::chrono::milliseconds(1000))) {
  //     std::cout << "Failed to vacuum the object." << std::endl;
  //     return -1;
  // }

  this->first_sample_taken = false;
  // TODO: create a subscriber and callback so I can control the joint vel from a ros topic
  sub_command_ = node_handle.subscribe<sensor_msgs::JointState>(
                  "/relaxed_ik/joint_angle_solutions", 1, &HIROJointVelocityExampleController::jointCommandCb, this); 
  last_time_called = ros::Time::now().toSec();

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "HIROJointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("HIROJointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HIROJointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting state handle from interface: " << ex.what());
        return false;
  }

  return true;
}

void HIROJointVelocityExampleController::jointCommandCb(const sensor_msgs::JointState::ConstPtr& joint_pos_commands) {
    for (int i = 0; i < 7; i++) joint_positions_[i] = joint_pos_commands->position[i];
    this->callback_done_once = true;
    this->last_time_called = ros::Time::now().toSec();
    std::cout << this->last_time_called << std::endl;
}

void HIROJointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void HIROJointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  // Currently just a proportional movement. 
  // I am going to keep track of the diffrence so I can make this a PI loss first.
  // Original -> 0.8
  // increasing p makes it feel a little more snappy. 
  //The following are okay with no overshoot: 1.5, 2.0, 3.0
  // TODO: We need to find the reason why there is a little grinding. 
  // Most likely need to smooth out some variable
  elapsed_time_ += period;
  if ((ros::Time::now().toSec() - this->last_time_called) > 3) {
        for (int i = 0; i < 7; i++) velocity_joint_handles_[i].setCommand(0.0);
        std::cout << "Set Vel to ZERO ******" << std::endl;

  } else if(this->callback_done_once) {  // If command recieved, send the command to the controller
        for (int i = 0; i < 7; i++) {
            this->curr_error_[i] = (joint_positions_[i] - robot_state.q[i]);
            double proportional = this->p_ * this->curr_error_[i];

            if(!this->first_sample_taken){
              this->first_sample_taken = true;
              velocity_joint_handles_[i].setCommand(proportional);
            }else{
              double deriv = this->d_ * (this->curr_error_[i] - this->prev_error_[i]);
              velocity_joint_handles_[i].setCommand(proportional);
            }

            // Always set the previous error value
            this->prev_error_[i] = (joint_positions_[i] - robot_state.q[i]);
        }
  }
  else{
      for (int i = 0; i < 7; i++) velocity_joint_handles_[i].setCommand(0.0);
  }
}

void HIROJointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HIROJointVelocityExampleController,
                       controller_interface::ControllerBase)
