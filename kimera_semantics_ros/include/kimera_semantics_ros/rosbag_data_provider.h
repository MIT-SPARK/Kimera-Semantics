/**
 * @file   RosBagDataProvider.h
 * @brief  Parse rosbag data for Kimera-Semantics.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace kimera {

using Timestamp = ros::Time;

struct RosbagData {
  std::vector<sensor_msgs::ImageConstPtr> depth_imgs_;
  std::vector<sensor_msgs::ImageConstPtr> semantic_imgs_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
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
  // The parsed rosbag data, first call initialize (parse) to populate this struct
  RosbagData rosbag_data_;

 private:
  // Parse rosbag data
  bool parseRosbag(const std::string& bag_path, RosbagData* rosbag_data);

  void publishRosbagInfo(const Timestamp& timestamp);

  void publishClock(const Timestamp& timestamp) const;

  void publishInputs(const Timestamp& timestamp_kf);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;


  std::string rosbag_path_;
  std::string depth_imgs_topic_;
  std::string semantic_imgs_topic_;
  std::string left_cam_info_topic_;

  ros::Publisher clock_pub_;
  ros::Publisher depth_img_pub_;
  ros::Publisher semantic_img_pub_;

  Timestamp timestamp_last_frame_;
  Timestamp timestamp_last_kf_;
  Timestamp timestamp_last_gt_;

  // Frame indices
  size_t k_;
  size_t k_last_kf_;
  size_t k_last_gt_;
};

}  // namespace VIO
