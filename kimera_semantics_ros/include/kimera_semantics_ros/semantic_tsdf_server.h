/**
 * @file   semantic_tsdf_server.h
 * @brief  Semantic TSDF Server to interface with ROS
 * @author Antoni Rosinol
 */
#pragma once

#include <ros/ros.h>

#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/tsdf_server.h>

#include "kimera_semantics/semantic_mesh_integrator.h"
#include "kimera_semantics/semantic_tsdf_integrator.h"
#include "kimera_semantics/semantic_voxel.h"
#include "kimera_semantics_ros/ros_params.h"

namespace kimera {

class SemanticTsdfServer : public vxb::TsdfServer {
 public:
  SemanticTsdfServer(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  SemanticTsdfServer(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     const vxb::TsdfMap::Config& config,
                     const vxb::TsdfIntegratorBase::Config& integrator_config,
                     const vxb::MeshIntegratorConfig& mesh_config);

  virtual ~SemanticTsdfServer() = default;

 protected:
  static std::string getSemanticLabelToColorCsvFilepathFromRosParam(
      const ros::NodeHandle& nh);

  // Configs.
  SemanticTsdfIntegrator::SemanticConfig semantic_config_;
  SemanticMeshIntegrator::SemanticMeshConfig semantic_mesh_config_;

  // Layers.
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_layer_;

  // Map from semantic label to actual color.
  const SemanticLabel2Color semantic_label_to_color_;
};

}  // Namespace kimera
