/**
 * @file   kimera_semantics_rosbag.cpp
 * @brief  Main for feeding a parsed rosbag to Kimera-Semantics
 * and generate a PLY mesh and serialize a TSDF layer.
 * @author Antoni Rosinol
 */

#include <ros/ros.h>

#include <glog/logging.h>

#include "kimera_semantics_ros/rosbag_data_provider.h"
#include "kimera_semantics_ros/semantic_tsdf_server.h"
#include "kimera_semantics_ros/depth_map_to_pointcloud.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_semantics");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string depth_cam_frame_id_;
  std::string base_link_frame_id_;
  std::string world_frame_id_;
  CHECK(nh_private.getParam("sensor_frame", depth_cam_frame_id_));
  CHECK(nh_private.getParam("base_link_frame", base_link_frame_id_));
  CHECK(nh_private.getParam("world_frame", world_frame_id_));

  std::string tsdf_filename;
  CHECK(nh_private.getParam("tsdf_filename", tsdf_filename));

  kimera::SemanticTsdfServer node(nh, nh_private);
  kimera::RosbagDataProvider rosbag;
  rosbag.initialize();
  CHECK_NOTNULL(rosbag.rosbag_data_);
  VLOG(1) << "All tfs as  STRING: \n"
          << rosbag.rosbag_data_->tf_listener_.allFramesAsString();
  VLOG(1) << "TF Listener Cached Length: "
          << rosbag.rosbag_data_->tf_listener_.getCacheLength().toSec();

  kimera::PointCloudFromDepth pcl_from_depth;
  const sensor_msgs::CameraInfoConstPtr& cam_info =
      rosbag.rosbag_data_->cam_info_;

  ros::Publisher pcl_pub;
  pcl_pub = nh.advertise<kimera::PointCloud>("pcl", 10, true);

  size_t n = rosbag.rosbag_data_->depth_imgs_.size();
  CHECK_EQ(rosbag.rosbag_data_->depth_imgs_.size(),
           rosbag.rosbag_data_->semantic_imgs_.size());
  for (size_t i = 0u; i < n; i++) {
    LOG(INFO) << "Processing pointcloud: " << i << " / " << n;
    const sensor_msgs::ImageConstPtr& depth_img =
        rosbag.rosbag_data_->depth_imgs_.at(i);
    const sensor_msgs::ImageConstPtr& semantic_img =
        rosbag.rosbag_data_->semantic_imgs_.at(i);
    CHECK_EQ(depth_img->header.stamp, semantic_img->header.stamp)
        << "Depth and Semantic Img timestamps do not match:\n"
        << "- Depth timestamp: " << depth_img->header.stamp.toSec() << '\n'
        << "- Semantic timestamp: " << semantic_img->header.stamp.toSec();
    // Programatically build the semantic pointcloud
    kimera::PointCloud::Ptr pcl =
        pcl_from_depth.imageCb(depth_img, semantic_img, cam_info);
    CHECK(pcl);
    pcl_pub.publish(pcl);

    LOG(INFO) << "PCL size: " << pcl->data.size();

    // Feed semantic pointcloud to KS.
    voxblox::Transformation T_G_B;
    if (kimera::lookupTransformTf(rosbag.rosbag_data_->tf_listener_,
                                  base_link_frame_id_,
                                  world_frame_id_,
                                  depth_img->header.stamp,
                                  &T_G_B)) {
      voxblox::Transformation T_B_C;
      tf::transformTFToKindr(rosbag.rosbag_data_->camera_to_base_link_tf_static_,
                             &T_B_C);
      voxblox::Transformation T_G_C = T_G_B * T_B_C;
      node.processPointCloudMessageAndInsert(pcl, T_G_C, false);
    } else {
      LOG(ERROR) << "Couldn't find tf for given pointcloud...";
    }

    if (!ros::ok()) {
      LOG(ERROR) << "ROS died, finishing KS processing.";
      return EXIT_FAILURE;
    } else {
      ros::spinOnce();
    }
  }

  // Generates mesh and saves the ply file if mesh_filename ROS param is given
  node.generateMesh();
  // Saves the TSDF layer
  node.saveMap(tsdf_filename);

  ros::spin();

  return EXIT_SUCCESS;
}
