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
}

std::string SemanticTsdfServer::getSemanticLabelToColorCsvFilepathFromRosParam(
    const ros::NodeHandle& nh) {
  std::string path = "semantics2labels.csv";
  nh.param("semantic_label_2_color_csv_filepath", path, path);
  return path;
}

}  // Namespace kimera
