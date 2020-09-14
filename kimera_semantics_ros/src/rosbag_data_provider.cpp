/**
 * @file   RosBagDataProvider.cpp
 * @brief  Parse rosbag data for Kimera-Semantics.
 * @author Antoni Rosinol
 */

#include "kimera_semantics_ros/rosbag_data_provider.h"

#include <glog/logging.h>

#include <rosgraph_msgs/Clock.h>

namespace kimera {

RosbagDataProvider::RosbagDataProvider()
    : nh_(),
      nh_private_("~"),
      rosbag_data_(),
      rosbag_path_(""),
      depth_imgs_topic_(""),
      semantic_imgs_topic_(""),
      left_cam_info_topic_(""),
      clock_pub_(),
      depth_img_pub_(),
      semantic_img_pub_(),
      timestamp_last_frame_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_kf_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_gt_(std::numeric_limits<Timestamp>::min()),
      k_(0u),
      k_last_kf_(0u),
      k_last_gt_(0u) {
  CHECK(nh_private_.getParam("rosbag_path", rosbag_path_));
  CHECK(nh_private_.getParam("depth_cam_topic", depth_imgs_topic_));
  CHECK(nh_private_.getParam("semantic_cam_topic", semantic_imgs_topic_));
  CHECK(nh_private_.getParam("left_cam_info_topic", left_cam_info_topic_));

  LOG(INFO) << "Constructing RosbagDataProvider from path: \n"
            << " - Rosbag Path: " << rosbag_path_.c_str() << '\n'
            << "With ROS topics: \n"
            << " - Depth cam: " << depth_imgs_topic_.c_str() << '\n'
            << " - Semantic cam: " << semantic_imgs_topic_.c_str() << '\n'
            << " - Cam info: " << left_cam_info_topic_.c_str();

  CHECK(!rosbag_path_.empty());
  CHECK(!depth_imgs_topic_.empty());
  CHECK(!semantic_imgs_topic_.empty());

  // Ros publishers specific to rosbag data provider
  static constexpr size_t kQueueSize = 10u;

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", kQueueSize);
  depth_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(depth_imgs_topic_, kQueueSize);
  semantic_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(semantic_imgs_topic_, kQueueSize);
}

void RosbagDataProvider::initialize() {
  CHECK_EQ(k_, 0u);
  LOG(INFO) << "Initialize Rosbag Data Provider.";
  // Parse data from rosbag first thing:
  CHECK(parseRosbag(rosbag_path_, &rosbag_data_));
}

bool RosbagDataProvider::parseRosbag(const std::string& bag_path,
                                     RosbagData* rosbag_data) {
  LOG(INFO) << "Parsing rosbag data.";
  CHECK_NOTNULL(rosbag_data);

  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Generate list of topics to parse:
  std::vector<std::string> topics;
  topics.push_back(depth_imgs_topic_);
  topics.push_back(semantic_imgs_topic_);
  topics.push_back(left_cam_info_topic_);

  // Query rosbag for given topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // For some datasets, we have duplicated measurements for the same time.
  size_t kMaxMsgs = 101;
  size_t i = 0;
  for (const rosbag::MessageInstance& msg : view) {
    if (i > kMaxMsgs) break;
    i++;

    LOG(INFO) << "Rosbag processing: " << msg.getTime() - view.getBeginTime()
              << " / " << view.getEndTime() - view.getBeginTime();

    const std::string& msg_topic = msg.getTopic();

    // Check if msg is an image.
    sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
    if (img_msg != nullptr) {
      // Check left or right image.
      if (msg_topic == depth_imgs_topic_) {
        rosbag_data->depth_imgs_.push_back(img_msg);
      } else if (msg_topic == semantic_imgs_topic_) {
        rosbag_data->semantic_imgs_.push_back(img_msg);
      } else {
        LOG(WARNING) << "Img with unexpected topic: " << msg_topic;
      }
      continue;
    }

    sensor_msgs::CameraInfoConstPtr cam_info =
        msg.instantiate<sensor_msgs::CameraInfo>();
    if (cam_info != nullptr) {
      if (msg_topic == left_cam_info_topic_) {
        rosbag_data->cam_info_ = cam_info;
      }
      continue;
    } else {
      LOG(ERROR) << "Could not find the type of this rosbag msg from topic:\n"
                 << msg_topic;
    }

    if (!ros::ok()) return false;
  }
  bag.close();

  // Sanity check:
  LOG_IF(FATAL, rosbag_data->depth_imgs_.size() == 0)
      << "No depth images parsed from rosbag.";
  LOG_IF(FATAL, rosbag_data->semantic_imgs_.size() == 0)
      << "No semantic images  parsed from rosbag.";
  LOG_IF(FATAL,
         rosbag_data->depth_imgs_.size() != rosbag_data->semantic_imgs_.size())
      << "Unequal number of depth and semantic images.";
  LOG(INFO) << "Finished parsing rosbag data.";
  return true;
}

void RosbagDataProvider::publishRosbagInfo(const Timestamp& timestamp) {
  publishInputs(timestamp);
  publishClock(timestamp);
}

void RosbagDataProvider::publishClock(const Timestamp& timestamp) const {
  rosgraph_msgs::Clock clock;
  clock.clock = timestamp;
  clock_pub_.publish(clock);
}

void RosbagDataProvider::publishInputs(const Timestamp& timestamp_kf) {
  // Publish left and right images:
  if (k_last_kf_ < rosbag_data_.depth_imgs_.size() &&
      k_last_kf_ < rosbag_data_.semantic_imgs_.size()) {
    while (timestamp_last_kf_ < timestamp_kf &&
           k_last_kf_ < rosbag_data_.depth_imgs_.size() &&
           k_last_kf_ < rosbag_data_.semantic_imgs_.size()) {
      depth_img_pub_.publish(rosbag_data_.depth_imgs_.at(k_last_kf_));
      semantic_img_pub_.publish(rosbag_data_.semantic_imgs_.at(k_last_kf_));
      timestamp_last_kf_ =
          rosbag_data_.depth_imgs_.at(k_last_kf_)->header.stamp;
      k_last_kf_++;
    }
  }
}

}  // namespace VIO
