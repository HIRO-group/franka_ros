# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "/home/caleb/ros_relaxed_ik_ws/src/franka_ros/franka_gazebo/include;/home/caleb/libfranka/include;/usr/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3".split(';') if "/home/caleb/ros_relaxed_ik_ws/src/franka_ros/franka_gazebo/include;/home/caleb/libfranka/include;/usr/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;gazebo_ros_control;kdl_parser;boost_sml;controller_manager;controller_interface;control_toolbox;pluginlib;hardware_interface;transmission_interface;joint_limits_interface;eigen_conversions;urdf;angles;franka_hw;franka_msgs;franka_gripper;franka_example_controllers".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_hw_sim;-lfranka_gripper_sim;/home/caleb/libfranka/build/libfranka.so.0.13.3;-lorocos-kdl".split(';') if "-lfranka_hw_sim;-lfranka_gripper_sim;/home/caleb/libfranka/build/libfranka.so.0.13.3;-lorocos-kdl" != "" else []
PROJECT_NAME = "franka_gazebo"
PROJECT_SPACE_DIR = "/home/caleb/ros_relaxed_ik_ws/devel"
PROJECT_VERSION = "0.10.1"
