/**
 * @file helper_functions.h
 *
 * @brief Helper functions for example controller code.
 *
 * @author Nataliya Nechyporenko
 *
 */

#pragma once

#include "std_msgs/Float64.h"

inline std_msgs::Float64 toROSType(double value) {
  std_msgs::Float64 msg;
  msg.data = value;
  return msg;
}