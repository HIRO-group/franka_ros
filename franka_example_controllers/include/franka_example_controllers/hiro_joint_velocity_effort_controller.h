// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

// Added in order to allow effort interface
#include <franka_hw/franka_model_interface.h>
#include <franka_example_controllers/JointTorqueComparison.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>


#include <Eigen/Core>




namespace franka_example_controllers {

class HIROVelocityEffortController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface,
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaModelInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  ros::Duration elapsed_time_;

  ros::Publisher command_pub1;
  std::vector<ros::Publisher> pubs;
  std::vector<ros::Publisher> pubs_fake;


  std::array<double, 7> prev_qd;
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;


  std::array<double, 7> p_gains{50, 50, 50, 20, 20, 20, 10};
  std::array<double, 7> i_gains{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 7> d_gains{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::array<double, 7> i_errors{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  Eigen::VectorXd r{6};
  Eigen::VectorXd t_c{7};

  Eigen::VectorXd cum_sum{7};

  // Added to allow for effort interface
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
  // added to allow receiving ros messages for commands.

  ros::Subscriber sub_command_;
  std::vector<double> commanded_joint_velocities;

  void jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_velocity_commands);

};

}  // namespace franka_example_controllers
