/**
 * @file   ros_params.h
 * @brief  Helper functions to parse ROS params
 * @author Antoni Rosinol
 */

#pragma once

#include <ros/ros.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_integrator_base.h"

namespace kimera {

inline std::string
getSemanticTsdfIntegratorTypeFromRosParam(const ros::NodeHandle& nh_private) {
  // Get semantic tsdf integrator type, by default using "fast"
  // (could be "merged").
  std::string semantic_tsdf_integrator_type = "fast";
  nh_private.param("semantic_tsdf_integrator_type",
                   semantic_tsdf_integrator_type,
                   semantic_tsdf_integrator_type);
  return semantic_tsdf_integrator_type;
}

inline SemanticIntegratorBase::SemanticConfig
getSemanticTsdfIntegratorConfigFromRosParam(const ros::NodeHandle& nh_private) {
  SemanticIntegratorBase::SemanticConfig semantic_config;

  // Get semantic meas prob
  double semantic_measurement_probability =
      semantic_config.semantic_measurement_probability_;
  nh_private.param("semantic_measurement_probability",
                   semantic_measurement_probability,
                   semantic_measurement_probability);
  semantic_config.semantic_measurement_probability_ =
      static_cast<SemanticProbability>(semantic_measurement_probability);

  // Get semantic color mode
  std::string color_mode = "color";
  nh_private.param("semantic_color_mode", color_mode, color_mode);
  if (color_mode == "color") {
    semantic_config.color_mode = ColorMode::kColor;
  } else if (color_mode == "semantic") {
    semantic_config.color_mode = ColorMode::kSemantic;
  } else if (color_mode == "semantic_probability") {
    semantic_config.color_mode = ColorMode::kSemanticProbability;
  } else {
    LOG(FATAL) << "Unknown semantic color mode: " << color_mode;
  }

  return semantic_config;
}

}  // namespace kimera
