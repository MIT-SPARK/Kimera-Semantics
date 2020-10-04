/**
 * @file   kimera_semantics_rosbag.cpp
 * @brief  Main for feeding a parsed rosbag to Kimera-Semantics
 * @author Antoni Rosinol
 */

#include <ros/ros.h>

#include "kimera_semantics_ros/rosbag_data_provider.h"
#include "kimera_semantics_ros/semantic_tsdf_server.h"

#include <cv_bridge/cv_bridge.h>
#include <depth_image_proc/depth_traits.h>
#include <glog/logging.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <minkindr_conversions/kindr_tf.h>

namespace kimera {

typedef sensor_msgs::PointCloud2 PointCloud;

class PointCloudFromDepth {
 public:
  template <typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const PointCloud::Ptr& cloud_msg,
               int red_offset,
               int green_offset,
               int blue_offset,
               int color_step);

  PointCloud::Ptr imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                          const sensor_msgs::ImageConstPtr& rgb_msg_in,
                          const sensor_msgs::CameraInfoConstPtr& info_msg) {
    // Check for bad inputs
    if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
      LOG(ERROR)
          << "Depth image frame id [%s] doesn't match RGB image frame id [%s]"
          << depth_msg->header.frame_id.c_str()
          << rgb_msg_in->header.frame_id.c_str();
      return nullptr;
    }

    // Update camera model
    model_.fromCameraInfo(info_msg);

    // Check if the input image has to be resized
    sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
    if (depth_msg->width != rgb_msg->width ||
        depth_msg->height != rgb_msg->height) {
      sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
      info_msg_tmp.width = depth_msg->width;
      info_msg_tmp.height = depth_msg->height;
      float ratio = float(depth_msg->width) / float(rgb_msg->width);
      info_msg_tmp.K[0] *= ratio;
      info_msg_tmp.K[2] *= ratio;
      info_msg_tmp.K[4] *= ratio;
      info_msg_tmp.K[5] *= ratio;
      info_msg_tmp.P[0] *= ratio;
      info_msg_tmp.P[2] *= ratio;
      info_msg_tmp.P[5] *= ratio;
      info_msg_tmp.P[6] *= ratio;
      model_.fromCameraInfo(info_msg_tmp);

      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return nullptr;
      }
      cv_bridge::CvImage cv_rsz;
      cv_rsz.header = cv_ptr->header;
      cv_rsz.encoding = cv_ptr->encoding;
      cv::resize(cv_ptr->image.rowRange(0, depth_msg->height / ratio),
                 cv_rsz.image,
                 cv::Size(depth_msg->width, depth_msg->height));
      if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) ||
          (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) ||
          (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8)) {
        rgb_msg = cv_rsz.toImageMsg();
      } else {
        rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(),
                                      sensor_msgs::image_encodings::RGB8)
                      ->toImageMsg();
      }

      // NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB
      // resolution (%ux%u)",
      //                       depth_msg->width, depth_msg->height,
      //                       rgb_msg->width, rgb_msg->height);
      // return;
    } else {
      rgb_msg = rgb_msg_in;
    }

    // Supported color encodings: RGB8, BGR8, MONO8
    int red_offset, green_offset, blue_offset, color_step;
    if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      red_offset = 0;
      green_offset = 1;
      blue_offset = 2;
      color_step = 3;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      red_offset = 2;
      green_offset = 1;
      blue_offset = 0;
      color_step = 3;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
      red_offset = 0;
      green_offset = 0;
      blue_offset = 0;
      color_step = 1;
    } else {
      try {
        rgb_msg =
            cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)
                ->toImageMsg();
      } catch (cv_bridge::Exception& e) {
        LOG(ERROR) << "Unsupported encoding [" << rgb_msg->encoding.c_str()
                   << "]: " << e.what();
        return nullptr;
      }
      red_offset = 0;
      green_offset = 1;
      blue_offset = 2;
      color_step = 3;
    }

    // Allocate new point cloud message
    PointCloud::Ptr cloud_msg(new PointCloud);
    cloud_msg->header = depth_msg->header;  // Use depth image time stamp
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      convert<uint16_t>(depth_msg,
                        rgb_msg,
                        cloud_msg,
                        red_offset,
                        green_offset,
                        blue_offset,
                        color_step);
    } else if (depth_msg->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      convert<float>(depth_msg,
                     rgb_msg,
                     cloud_msg,
                     red_offset,
                     green_offset,
                     blue_offset,
                     color_step);
    } else {
      LOG_EVERY_N(ERROR, 5) << "Depth image has unsupported encoding: "
                            << depth_msg->encoding.c_str();
      return nullptr;
    }

    return cloud_msg;
  }

 public:
  image_geometry::PinholeCameraModel model_;
};

template <typename T>
void PointCloudFromDepth::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                  const sensor_msgs::ImageConstPtr& rgb_msg,
                                  const PointCloud::Ptr& cloud_msg,
                                  int red_offset,
                                  int green_offset,
                                  int blue_offset,
                                  int color_step) {
  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for
  // computing (X,Y)
  double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1));
  float constant_x = unit_scaling / model_.fx();
  float constant_y = unit_scaling / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0; v < int(cloud_msg->height);
       ++v, depth_row += row_step, rgb += rgb_skip) {
    for (int u = 0; u < int(cloud_msg->width); ++u,
             rgb += color_step,
             ++iter_x,
             ++iter_y,
             ++iter_z,
             ++iter_a,
             ++iter_r,
             ++iter_g,
             ++iter_b) {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!depth_image_proc::DepthTraits<T>::valid(depth)) {
        *iter_x = *iter_y = *iter_z = bad_point;
      } else {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}

bool lookupTransformTf(const tf::TransformListener& tf_listener,
                       const std::string& from_frame,
                       const std::string& to_frame,
                       const ros::Time& timestamp,
                       voxblox::Transformation* transform) {
  CHECK_NOTNULL(transform);
  tf::StampedTransform tf_transform;

  LOG_IF(ERROR, !tf_listener.frameExists(from_frame))
      << "Frame id: " << from_frame << " does not exist in TF tree.";
  LOG_IF(ERROR, !tf_listener.frameExists(to_frame))
      << "Frame id: " << to_frame << " does not exist in TF tree.";

  // to_frame === target_frame, from_frame === source_frame
  std::string error_msg;
  if (!tf_listener.canTransform(to_frame, from_frame, timestamp, &error_msg)) {
    LOG(ERROR) << "Can't find transform from frame " << from_frame.c_str()
               << " to frame " << to_frame.c_str() << " at time "
               << timestamp.toSec() << '\n'
               << "Error is: \n"
               << error_msg.c_str();
    LOG_IF(
        ERROR,
        !tf_listener.canTransform(from_frame, to_frame, timestamp, &error_msg))
        << "NOR it can find transform from frame " << to_frame.c_str()
        << " to frame " << from_frame.c_str() << " at time "
        << timestamp.toSec() << '\n'
        << "Error is: \n"
        << error_msg.c_str();
    return false;
  }

  try {
    tf_listener.lookupTransform(to_frame, from_frame, timestamp, tf_transform);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  } catch (tf::LookupException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  } catch (tf::ConnectivityException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  } catch (tf::ExtrapolationException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  } catch (tf2::InvalidArgumentException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}
}

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
  LOG(ERROR) << "All tfs as  STRING: \n"
             << rosbag.rosbag_data_->tf_listener_.allFramesAsString();
  LOG(ERROR) << "TF Listener Cached Length: "
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
