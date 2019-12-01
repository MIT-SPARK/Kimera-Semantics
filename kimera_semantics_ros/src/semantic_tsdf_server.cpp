// NOTE: Most code is derived from voxblox: github.com/ethz-asl/voxblox
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @file   semantic_tsdf_server.cpp
 * @brief  Semantic TSDF Server to interface with ROS
 * @author Antoni Rosinol
 */

#include "kimera_semantics_ros/semantic_tsdf_server.h"

namespace kimera {

SemanticTsdfServer::SemanticTsdfServer(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : SemanticTsdfServer(nh,
                         nh_private,
                         vxb::getTsdfMapConfigFromRosParam(nh_private),
                         vxb::getTsdfIntegratorConfigFromRosParam(nh_private),
                         vxb::getMeshIntegratorConfigFromRosParam(nh_private)) {
}

SemanticTsdfServer::SemanticTsdfServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const vxb::TsdfMap::Config& config,
    const vxb::TsdfIntegratorBase::Config& integrator_config,
    const vxb::MeshIntegratorConfig& mesh_config)
    : vxb::TsdfServer(nh, nh_private, config, integrator_config, mesh_config),
      semantic_config_(getSemanticTsdfIntegratorConfigFromRosParam(nh_private)),
      semantic_layer_(nullptr),
      semantic_label_to_color_(
          getSemanticLabelToColorCsvFilepathFromRosParam(nh_private)) {
  /// Semantic layer
  semantic_layer_.reset(new vxb::Layer<SemanticVoxel>(
      config.tsdf_voxel_size, config.tsdf_voxels_per_side));
  /// Semantic configuration
  semantic_config_.semantic_label_color_map_ =
      semantic_label_to_color_.semantic_label_to_color_map_;
  semantic_config_.color_to_semantic_label_map_ =
      semantic_label_to_color_.color_to_semantic_label_;
  /// Replace the TSDF integrator by the SemanticTsdfIntegrator
  tsdf_integrator_ =
      SemanticTsdfIntegratorFactory::create(
        getSemanticTsdfIntegratorTypeFromRosParam(nh_private),
        integrator_config,
        semantic_config_,
        tsdf_map_->getTsdfLayerPtr(),
        semantic_layer_.get());
  CHECK(tsdf_integrator_);
}

std::string SemanticTsdfServer::getSemanticLabelToColorCsvFilepathFromRosParam(
    const ros::NodeHandle& nh) {
  std::string path = "semantics2labels.csv";
  nh.param("semantic_label_2_color_csv_filepath", path, path);
  return path;
}

}  // Namespace kimera
