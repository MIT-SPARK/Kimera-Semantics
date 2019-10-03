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

  double semantic_truncation_distance_factor =
      semantic_config.semantic_truncation_distance_factor_;
  nh_private.param("semantic_truncation_distance_factor",
                   semantic_truncation_distance_factor,
                   semantic_truncation_distance_factor);
  semantic_config.semantic_truncation_distance_factor_ =
      static_cast<vxb::FloatingPoint>(semantic_truncation_distance_factor);

  return semantic_config;
}

inline SemanticMeshIntegrator::SemanticMeshConfig
getSemanticMeshConfigFromRosParam(const ros::NodeHandle& nh_private) {
  SemanticMeshIntegrator::SemanticMeshConfig semantic_mesh_config;

  double min_probability = semantic_mesh_config.min_probability;
  nh_private.param(
      "semantic_mesh_min_probability", min_probability, min_probability);
  semantic_mesh_config.min_probability =
      static_cast<SemanticProbability>(min_probability);

  // nh_private.param("semantic_mesh_color_mode",
  //                 semantic_mesh_config.color_mode,
  //                 semantic_mesh_config.color_mode);

  return semantic_mesh_config;
}

}  // namespace kimera
