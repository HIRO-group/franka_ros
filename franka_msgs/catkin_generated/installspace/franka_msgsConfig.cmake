# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(franka_msgs_CONFIG_INCLUDED)
  return()
endif()
set(franka_msgs_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(franka_msgs_SOURCE_PREFIX /home/caleb/ros_relaxed_ik_ws/src/franka_ros/franka_msgs)
  set(franka_msgs_DEVEL_PREFIX /home/caleb/ros_relaxed_ik_ws/devel)
  set(franka_msgs_INSTALL_PREFIX "")
  set(franka_msgs_PREFIX ${franka_msgs_DEVEL_PREFIX})
else()
  set(franka_msgs_SOURCE_PREFIX "")
  set(franka_msgs_DEVEL_PREFIX "")
  set(franka_msgs_INSTALL_PREFIX /home/caleb/ros_relaxed_ik_ws/install)
  set(franka_msgs_PREFIX ${franka_msgs_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'franka_msgs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(franka_msgs_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include " STREQUAL " ")
  set(franka_msgs_INCLUDE_DIRS "")
  set(_include_dirs "include")
  if(NOT "https://github.com/frankaemika/franka_ros/issues " STREQUAL " ")
    set(_report "Check the issue tracker 'https://github.com/frankaemika/franka_ros/issues' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "http://wiki.ros.org/franka_msgs " STREQUAL " ")
    set(_report "Check the website 'http://wiki.ros.org/franka_msgs' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Franka Emika GmbH <support@franka.de>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${franka_msgs_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'franka_msgs' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'franka_msgs' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(franka_msgs_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND franka_msgs_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND franka_msgs_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT franka_msgs_NUM_DUMMY_TARGETS)
      set(franka_msgs_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::franka_msgs::wrapped-linker-option${franka_msgs_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR franka_msgs_NUM_DUMMY_TARGETS "${franka_msgs_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::franka_msgs::wrapped-linker-option${franka_msgs_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND franka_msgs_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND franka_msgs_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND franka_msgs_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/caleb/ros_relaxed_ik_ws/install/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/pilz_industrial_motion_planner/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/pilz_industrial_motion_planner_testutils/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/panda_simulation/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_calibration_gui/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_visual_tools/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_control_interface/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_simple_controller_manager/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_setup_assistant/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_servo/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_visualization/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_planners_chomp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_planning_interface/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_benchmarks/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_warehouse/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_robot_interaction/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_perception/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_manipulation/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_move_group/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_planners_ompl/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_kinematics/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_fake_controller_manager/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_planning/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros_occupancy_map_monitor/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_prbt_ikfast_manipulator_plugin/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_chomp_optimizer_adapter/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_calibration_plugins/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/chomp_motion_planner/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_core/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_panda_description/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_commander/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_gazebo/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_example_controllers/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_control/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_hw/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/xacro/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/visualization_marker_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/urdf_tutorial/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/srdfdom/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rviz_visual_tools/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rviz_python_tutorial/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rviz_plugin_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_rviz/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/librviz_tutorial/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rviz/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/robot_state_publisher/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/kdl_parser/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/urdf/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/turtle_tf2/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/turtle_tf/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/turtle_actionlib/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/turtlesim/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf_conversions/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2_kdl/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/laser_geometry/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/interactive_marker_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/interactive_markers/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2_geometry_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2_ros/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/stereo_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_image_view/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/map_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/image_transport/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_visualization/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_gripper/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/cv_bridge/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/sensor_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_launch/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roswtf/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/nodelet_topic_tools/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostic_analysis/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/actionlib_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/actionlib/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbag/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/topic_tools/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2_py/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/smach_ros/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/self_test/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_reconfigure/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_gui_py/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_gui_cpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rostopic/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rospy_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosnode/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosmsg/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbag_storage/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/message_filters/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/joint_state_publisher/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/filters/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/dynamic_reconfigure/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostic_common_diagnostics/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostic_updater/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostic_aggregator/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosout/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscpp_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbag_to_csv/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_panda_path_plotter/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/relaxed_ik_ros1/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/pluginlib_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/nodelet_tutorial_math/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/nodelet/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_lock_unlock/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/bondpy/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/bondcpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/xmlrpcpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/webkit_dependency/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/viz/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/visualization_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/geometric_shapes/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/visualization_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/urdf_parser_plugin/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/control_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/trajectory_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/tf2_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/std_srvs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/smach_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/shape_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_py_common/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosgraph_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/nav_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/kdl_conversions/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/eigen_conversions/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/geometry_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostic_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/bond/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/actionlib_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/std_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/smclib/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/smach/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_web/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_topic/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_top/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_tf_tree/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_srv/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_shell/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_service_caller/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_runtime_monitor/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_robot_steering/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_robot_plugins/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_robot_monitor/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_robot_dashboard/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_py_console/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_publisher/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_pose_view/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_plot/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_nav_view/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_msg/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_moveit/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_logger_level/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_gui/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_graph/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_dep/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_console/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_common_plugins/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_bag_plugins/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_bag/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rqt_action/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rostest/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslz4/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosconsole_bridge/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/resource_retriever/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/qt_gui_cpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/pluginlib/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosconsole/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_description/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/angles/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosunit/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslaunch/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/python_qt_binding/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscpp_serialization/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rostime/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosservice/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rospy/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosparam/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslib/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rospack/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosmaster/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosmake/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslisp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslint/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roslang/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosgraph/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscreate/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscpp_traits/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/roscpp_core/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosclean/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbuild/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosboost_cfg/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbash/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/rosbag_migration_rule/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_environment/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_core/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_comm/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros_base/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/ros/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/robot/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/qwt_dependency/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/qt_gui_py_common/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/qt_gui/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/qt_dotgraph/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/panda_moveit_config/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/nodelet_core/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_runtime/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_ros/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_prbt_support/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_prbt_pg70_support/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_prbt_moveit_config/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_pr2_description/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_panda_moveit_config/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_fanuc_moveit_config/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_fanuc_description/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources_dual_panda_moveit_config/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_resources/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_plugins/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit_planners/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/moveit/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/mk/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/message_runtime/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/message_generation/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/media_export/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/joint_state_publisher_gui/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/gl_dependency/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/geometry_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/geometry/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/franka_ros/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/executive_smach/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/diagnostics/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/desktop/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/cpp_common/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/common_tutorials/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/common_msgs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/class_loader/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/cmake_modules/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/bond_core/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/genpy/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/gennodejs/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/genlisp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/geneus/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/gencpp/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/genmsg/lib;/home/caleb/ros_relaxed_ik_ws/devel_isolated/catkin/lib;/home/caleb/ros_relaxed_ik_ws/install_isolated/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(franka_msgs_LIBRARY_DIRS ${lib_path})
      list(APPEND franka_msgs_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'franka_msgs'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND franka_msgs_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(franka_msgs_EXPORTED_TARGETS "franka_msgs_generate_messages_cpp;franka_msgs_generate_messages_eus;franka_msgs_generate_messages_lisp;franka_msgs_generate_messages_nodejs;franka_msgs_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${franka_msgs_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "message_runtime;std_msgs;sensor_msgs;actionlib_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 franka_msgs_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${franka_msgs_dep}_FOUND)
      find_package(${franka_msgs_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${franka_msgs_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(franka_msgs_INCLUDE_DIRS ${${franka_msgs_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(franka_msgs_LIBRARIES ${franka_msgs_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${franka_msgs_dep}_LIBRARIES})
  _list_append_deduplicate(franka_msgs_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(franka_msgs_LIBRARIES ${franka_msgs_LIBRARIES})

  _list_append_unique(franka_msgs_LIBRARY_DIRS ${${franka_msgs_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(franka_msgs_EXPORTED_TARGETS ${${franka_msgs_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "franka_msgs-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${franka_msgs_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
