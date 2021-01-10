#include "kimera_semantics_ros/semantic_simulation_server.h"

namespace kimera {

SemanticSimulationServer::SemanticSimulationServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : vxb::SimulationServer(nh, nh_private),
      semantic_config_(
          getSemanticTsdfIntegratorConfigFromRosParam(nh_private)) {
  world_ = make_unique<SemanticSimulationWorld>();

  semantic_gt_.reset(
      new vxb::Layer<SemanticVoxel>(voxel_size_, voxels_per_side_));

  semantic_test_.reset(
      new vxb::Layer<SemanticVoxel>(voxel_size_, voxels_per_side_));

  tsdf_integrator_ =
      SemanticTsdfIntegratorFactory::create(SemanticTsdfIntegratorType::kFast,
                                            tsdf_integrator_->getConfig(),
                                            semantic_config_,
                                            tsdf_test_.get(),
                                            semantic_test_.get());
  // Ros ground-truth publisher.
  semantic_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
      "semantic_gt", 1, true);
  semantic_test_mesh_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
          "semantic_test_mesh", 1, true);
}

// The below two functions are taken from the TSDF Server...
bool SemanticSimulationServer::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response&
    /*response*/) {  // NOLINT
  bool success = loadTsdfMap(request.file_path);
  if (success) {
    static constexpr bool build_esdf_ = false;
    if (build_esdf_) {
      LOG(INFO) << "Building ESDF layer.";
      esdf_integrator_->setFullEuclidean(true);
      esdf_integrator_->updateFromTsdfLayerBatch();
      LOG(INFO) << "Done building ESDF layer.";
    }
    if (visualize_) {
      LOG(INFO) << "Visualize";
      SimulationServer::visualize();
    }
  } else {
    LOG(ERROR) << "Failed to load map!";
  }
  return success;
}

bool SemanticSimulationServer::loadTsdfMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = vxb::io::LoadBlocksFromFile(
      file_path,
      vxb::Layer<vxb::TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport,
      tsdf_test_.get());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  } else {
    LOG(INFO) << "Failed to load TSDF layer.";
  }
  return success;
}

bool SemanticSimulationServer::loadEsdfMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = vxb::io::LoadBlocksFromFile(
      file_path,
      vxb::Layer<vxb::EsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport,
      esdf_test_.get());
  if (success) {
    LOG(INFO) << "Successfully loaded ESDF layer.";
  } else {
    LOG(INFO) << "Failed to load ESDF layer.";
  }
  return success;
}

}  // namespace kimera
