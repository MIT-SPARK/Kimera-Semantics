/**
 * @file   RosBagDataProvider.h
 * @brief  Parse rosbag data for Kimera-Semantics.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <numeric>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

namespace kimera {

using Timestamp = ros::Time;

struct RosbagData {
 public:
  RosbagData(const ros::Duration& rosbag_duration)
      : depth_imgs_(),
        semantic_imgs_(),
        rgb_imgs_(),
        cam_info_(),
        camera_to_base_link_tf_static_(),
        tf_listener_(ros::Duration(rosbag_duration)) {}

 public:
  std::vector<sensor_msgs::ImageConstPtr> depth_imgs_;
  std::vector<sensor_msgs::ImageConstPtr> semantic_imgs_;
  std::vector<sensor_msgs::ImageConstPtr> rgb_imgs_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
  tf::StampedTransform camera_to_base_link_tf_static_;
  tf::TransformListener tf_listener_;
};

class RosbagDataProvider {
 public:
  RosbagDataProvider();
  virtual ~RosbagDataProvider() = default;

  /**
   * @brief initialize The Rosbag data provider: it will parse the Rosbag and
   * call Kimera-
   */
  void initialize();

 public:
  // The parsed rosbag data, first call initialize (parse) to populate this
  // struct
  std::unique_ptr<RosbagData> rosbag_data_;

 private:
  // Parse rosbag data
  bool parseRosbag(const std::string& bag_path);

  void publishRosbagInfo(const Timestamp& timestamp);

  void publishClock(const Timestamp& timestamp) const;

  void publishInputs(const Timestamp& timestamp_kf);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Topics
  std::string rosbag_path_;
  std::string depth_imgs_topic_;
  std::string semantic_imgs_topic_;
  std::string rgb_imgs_topic_;
  std::string left_cam_info_topic_;

  // Publishers
  ros::Publisher clock_pub_;
  ros::Publisher depth_img_pub_;
  ros::Publisher semantic_img_pub_;
  ros::Publisher rgb_img_pub_;

  // Frame IDs
  std::string sensor_frame_id_;
  std::string base_link_gt_frame_id_;
  std::string world_frame_id_;

  Timestamp timestamp_last_frame_;
  Timestamp timestamp_last_kf_;
  Timestamp timestamp_last_gt_;

  // Frame indices
  size_t k_;
  size_t k_last_kf_;
  size_t k_last_gt_;
};

}  // namespace VIO
