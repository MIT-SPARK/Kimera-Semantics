#pragma once

#include <voxblox_ros/simulation_server.h>

#include "kimera_semantics/color.h"
#include "kimera_semantics/semantic_tsdf_integrator_factory.h"
#include "kimera_semantics/semantic_voxel.h"
#include "kimera_semantics/simulation/semantic_simulation_world.h"
#include "kimera_semantics_ros/ros_params.h"

namespace kimera {

class SemanticSimulationServer : public vxb::SimulationServer {
 public:
  SemanticSimulationServer(const ros::NodeHandle& nh,
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
  }

 private:
  ros::Publisher semantic_gt_pub_;
  ros::Publisher semantic_gt_mesh_pub_;
  ros::Publisher semantic_tsdf_test_pub_;
  ros::Publisher semantic_test_mesh_pub_;

  SemanticIntegratorBase::SemanticConfig semantic_config_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_gt_;

  // Generated maps:
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_test_;

  // Integrators:
  std::unique_ptr<vxb::TsdfIntegratorBase> semantic_tsdf_integrator_;
};

}  // namespace kimera
