/**
 * @file   ros_params.h
 * @brief  Helper functions to parse ROS params
 * @author Antoni Rosinol
 */

#pragma once

#include <ros/ros.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_mesh_integrator.h"
#include "kimera_semantics/semantic_tsdf_integrator.h"

namespace kimera {

inline SemanticTsdfIntegrator::SemanticConfig
getSemanticTsdfIntegratorConfigFromRosParam(const ros::NodeHandle& nh_private) {
  SemanticTsdfIntegrator::SemanticConfig semantic_config;

  double semantic_measurement_probability =
      semantic_config.semantic_measurement_probability_;
  nh_private.param("semantic_measurement_probability",
                   semantic_measurement_probability,
                   semantic_measurement_probability);
  semantic_config.semantic_measurement_probability_ =
      static_cast<SemanticProbability>(semantic_measurement_probability);

  return semantic_config;
}

inline SemanticMeshIntegrator::SemanticMeshConfig
getSemanticMeshConfigFromRosParam(const ros::NodeHandle& nh_private) {
  SemanticMeshIntegrator::SemanticMeshConfig semantic_mesh_config;

  double min_probability = semantic_mesh_config.min_probability;
  nh_private.param("semantic_mesh_min_probability",
                   min_probability,
                   min_probability);
  semantic_mesh_config.min_probability =
      static_cast<SemanticProbability>(min_probability);

  std::string semantic_color_mode = "semantic";
  nh_private.param("semantic_mesh_color_mode",
                   semantic_color_mode,
                   semantic_color_mode);
  if (semantic_color_mode == "semantic") {
    semantic_mesh_config.color_mode =
        SemanticMeshIntegrator::ColorMode::kSemantic;
  } else if (semantic_color_mode == "semantic_probability") {
    semantic_mesh_config.color_mode =
        SemanticMeshIntegrator::ColorMode::kSemanticProbability;
  } else {
    ROS_FATAL_STREAM("Undefined semantic mesh coloring mode: "
                     << semantic_color_mode);
    ros::shutdown();
  }

  return semantic_mesh_config;
}

}  // namespace kimera
