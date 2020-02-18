/**
 * @file   ros_params.cpp
 * @brief  Helper functions to parse ROS params
 * @author Antoni Rosinol
 */

#include "kimera_semantics_ros/ros_params.h"

#include <string>

#include <ros/ros.h>

#include <glog/logging.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_integrator_base.h"

namespace kimera {

std::string
getSemanticTsdfIntegratorTypeFromRosParam(const ros::NodeHandle& nh_private) {
  // Get semantic tsdf integrator type, by default using "fast"
  // (could be "merged").
  std::string semantic_tsdf_integrator_type = "fast";
  nh_private.param("method",
                   semantic_tsdf_integrator_type,
                   semantic_tsdf_integrator_type);
  return semantic_tsdf_integrator_type;
}

std::string getSemanticLabelToColorCsvFilepathFromRosParam(
    const ros::NodeHandle& nh) {
  std::string path = "semantics2labels.csv";
  nh.param("semantic_label_2_color_csv_filepath", path, path);
  return path;
}

SemanticIntegratorBase::SemanticConfig
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

  // Get semantic map
  semantic_config.semantic_label_to_color_ =
      kimera::make_unique<SemanticLabel2Color>(
          getSemanticLabelToColorCsvFilepathFromRosParam(nh_private));

  std::vector<int> dynamic_labels;
  CHECK(nh_private.getParam("dynamic_semantic_labels", dynamic_labels));
  semantic_config.dynamic_labels_.clear();
  for (const auto& label : dynamic_labels) {
    semantic_config.dynamic_labels_.push_back(label);
  }

  return semantic_config;
}

}  // namespace kimera
