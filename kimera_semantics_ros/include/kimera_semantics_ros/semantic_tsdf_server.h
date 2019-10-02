#pragma once

#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/tsdf_server.h>

#include "kimera_semantics/semantic_voxel.h"
#include "kimera_semantics/semantic_mesh_integrator.h"
#include "kimera_semantics/semantic_tsdf_integrator.h"
#include "kimera_semantics_ros/ros_params.h"

namespace kimera {

class SemanticTsdfServer : public vxb::TsdfServer {
 public:
  SemanticTsdfServer(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
      : SemanticTsdfServer(nh,
                           nh_private,
                           vxb::getTsdfMapConfigFromRosParam(nh_private),
                           vxb::getTsdfIntegratorConfigFromRosParam(nh_private),
                           vxb::getMeshIntegratorConfigFromRosParam(nh_private)) {}

  SemanticTsdfServer(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     const vxb::TsdfMap::Config& config,
                     const vxb::TsdfIntegratorBase::Config& integrator_config,
                     const vxb::MeshIntegratorConfig& mesh_config)
      : vxb::TsdfServer(nh, nh_private, config, integrator_config, mesh_config),
        semantic_config_(
            getSemanticTsdfIntegratorConfigFromRosParam(nh_private)),
        semantic_mesh_config_(getSemanticMeshConfigFromRosParam(nh_private)),
        semantic_layer_(nullptr),
        semantic_label_to_color_(
            getSemanticLabelToColorCsvFilepathFromRosParam(nh_private)) {
    semantic_layer_.reset(new vxb::Layer<SemanticVoxel>(
        config.tsdf_voxel_size, config.tsdf_voxels_per_side));
    // Replace the TSDF integrator by the SemanticTsdfIntegrator
    semantic_config_.semantic_label_color_map_ =
        semantic_label_to_color_.semantic_label_to_color_;
    semantic_config_.color_to_semantic_label_map_ =
        semantic_label_to_color_.color_to_semantic_label_;
    tsdf_integrator_.reset(
        new SemanticTsdfIntegrator(integrator_config,
                                   semantic_config_,
                                   semantic_layer_.get(),
                                   tsdf_map_->getTsdfLayerPtr()));
    // Replace the Mesh integrator by the SemanticMeshIntegrator
    semantic_mesh_config_.semantic_label_color_map =
        semantic_label_to_color_.semantic_label_to_color_;
    mesh_integrator_.reset(
        new SemanticMeshIntegrator(mesh_config,
                                   semantic_mesh_config_,
                                   tsdf_map_->getTsdfLayerPtr(),
                                   semantic_layer_.get(),
                                   mesh_layer_.get()));
  };

  virtual ~SemanticTsdfServer() = default;

 protected:
  static std::string getSemanticLabelToColorCsvFilepathFromRosParam(
      const ros::NodeHandle& nh) {
    std::string path = "semantics2labels.csv";
    nh.param("semantic_label_2_color_csv_filepath", path, path);
    return path;
  }

  // Configs.
  SemanticTsdfIntegrator::SemanticConfig semantic_config_;
  SemanticMeshIntegrator::SemanticMeshConfig semantic_mesh_config_;

  // Layers.
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_layer_;

  // Map from semantic label to actual color.
  const SemanticLabel2Color semantic_label_to_color_;
};

}  // Namespace kimera
