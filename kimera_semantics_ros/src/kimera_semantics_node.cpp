/**
 * @file   kimera_semantics_node.cpp
 * @brief  Main for Kimera-Semantics
 * @author Antoni Rosinol
 */

#include "kimera_semantics_ros/semantic_tsdf_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_semantics");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::SemanticTsdfServer node(nh, nh_private);

  ros::spin();

  return EXIT_SUCCESS;
}
