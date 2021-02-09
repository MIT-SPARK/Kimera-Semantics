#pragma once

#include <iostream>
#include <vector>

#include <glog/logging.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/simulation_server.h>

#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/FilePathRequest.h>
#include <voxblox_msgs/FilePathResponse.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/color.h"
#include "kimera_semantics/semantic_tsdf_integrator_factory.h"
#include "kimera_semantics/semantic_voxel.h"
#include "kimera_semantics/simulation/semantic_simulation_world.h"
#include "kimera_semantics_ros/ros_params.h"

namespace kimera {

class SemanticSimulationServer : public vxb::SimulationServer {
 public:
  SemanticSimulationServer(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private);

  // The below two functions are taken from the TSDF Server...
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,
                       voxblox_msgs::FilePath::Response&);

  bool loadTsdfMap(const std::string& file_path);
  bool loadEsdfMap(const std::string& file_path);

 protected:
  ros::Publisher semantic_gt_pub_;
  ros::Publisher semantic_gt_mesh_pub_;
  ros::Publisher semantic_tsdf_test_pub_;
  ros::Publisher semantic_test_mesh_pub_;

  SemanticIntegratorBase::SemanticConfig semantic_config_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_gt_;

  // Generated maps:
  std::unique_ptr<vxb::Layer<SemanticVoxel>> semantic_test_;
};

}  // namespace kimera
