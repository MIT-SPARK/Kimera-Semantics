/**
 * @file   ros_params.h
 * @brief  Helper functions to parse ROS params
 * @author Antoni Rosinol
 */

#pragma once

#include <string>

#include <ros/ros.h>

#include <glog/logging.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_integrator_base.h"

namespace kimera {

std::string getSemanticTsdfIntegratorTypeFromRosParam(
    const ros::NodeHandle& nh_private);

std::string getSemanticLabelToColorCsvFilepathFromRosParam(
    const ros::NodeHandle& nh);

SemanticIntegratorBase::SemanticConfig
getSemanticTsdfIntegratorConfigFromRosParam(const ros::NodeHandle& nh_private);

}  // namespace kimera
