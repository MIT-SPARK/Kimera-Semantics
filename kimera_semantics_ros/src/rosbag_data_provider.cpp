/**
 * @file   RosBagDataProvider.cpp
 * @brief  Parse rosbag data for Kimera-Semantics.
 * @author Antoni Rosinol
 */

#include "kimera_semantics/common.h"
#include "kimera_semantics_ros/rosbag_data_provider.h"

#include <glog/logging.h>

#include <ros/duration.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <rosgraph_msgs/Clock.h>

namespace kimera {

RosbagDataProvider::RosbagDataProvider()
    : nh_(),
      nh_private_("~"),
      rosbag_data_(nullptr),
      rosbag_path_(""),
      depth_imgs_topic_(""),
      semantic_imgs_topic_(""),
      rgb_imgs_topic_(""),
      left_cam_info_topic_(""),
      clock_pub_(),
      depth_img_pub_(),
      semantic_img_pub_(),
      rgb_img_pub_(),
      timestamp_last_frame_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_kf_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_gt_(std::numeric_limits<Timestamp>::min()),
      k_(0u),
      k_last_kf_(0u),
      k_last_gt_(0u) {
  CHECK(nh_private_.getParam("rosbag_path", rosbag_path_));
  CHECK(nh_private_.getParam("depth_cam_topic", depth_imgs_topic_));
  CHECK(nh_private_.getParam("semantic_cam_topic", semantic_imgs_topic_));
  CHECK(nh_private_.getParam("rgb_cam_topic", rgb_imgs_topic_));
  CHECK(nh_private_.getParam("left_cam_info_topic", left_cam_info_topic_));

  CHECK(nh_private_.getParam("sensor_frame", sensor_frame_id_));
  CHECK(nh_private_.getParam("base_link_gt_frame", base_link_gt_frame_id_));
  CHECK(nh_private_.getParam("world_frame", world_frame_id_));

  LOG(INFO) << "Constructing RosbagDataProvider from path: \n"
            << " - Rosbag Path: " << rosbag_path_.c_str() << '\n'
            << "With ROS topics: \n"
            << " - Depth cam: " << depth_imgs_topic_.c_str() << '\n'
            << " - Semantic cam: " << semantic_imgs_topic_.c_str() << '\n'
            << " - RGB cam: " << rgb_imgs_topic_.c_str() << '\n'
            << " - Cam info: " << left_cam_info_topic_.c_str();

  CHECK(!rosbag_path_.empty());
  CHECK(!depth_imgs_topic_.empty());
  CHECK(!semantic_imgs_topic_.empty());
  CHECK(!rgb_imgs_topic_.empty());

  // Ros publishers specific to rosbag data provider
  static constexpr size_t kQueueSize = 10u;

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", kQueueSize);
  depth_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(depth_imgs_topic_, kQueueSize);
  semantic_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(semantic_imgs_topic_, kQueueSize);
  rgb_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(rgb_imgs_topic_, kQueueSize);
}

void RosbagDataProvider::initialize() {
  CHECK_EQ(k_, 0u);
  LOG(INFO) << "Initialize Rosbag Data Provider.";
  // Parse data from rosbag first thing:
  CHECK(parseRosbag(rosbag_path_));
}

bool RosbagDataProvider::parseRosbag(const std::string& bag_path) {
  LOG(INFO) << "Parsing rosbag data.";


  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Generate list of topics to parse:
  std::vector<std::string> topics;
  topics.push_back(depth_imgs_topic_);
  topics.push_back(semantic_imgs_topic_);
  topics.push_back(rgb_imgs_topic_);
  topics.push_back(left_cam_info_topic_);
  topics.push_back("/tf");
  topics.push_back("/tf_static");

  // Query rosbag for given topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Duration rosbag_duration = view.getEndTime() - view.getBeginTime();
  rosbag_data_ = kimera::make_unique<RosbagData>(rosbag_duration);

  // For some datasets, we have duplicated measurements for the same time.
  static constexpr bool kEarlyStopForDebug = false;
  for (const rosbag::MessageInstance& msg : view) {
    if (!nh_.ok() || !ros::ok() || ros::isShuttingDown()) return false;
    LOG(INFO) << "Rosbag processing: " << msg.getTime() - view.getBeginTime()
              << " / " << rosbag_duration;

    const std::string& msg_topic = msg.getTopic();

    // Check if msg is an image.
    sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
    if (img_msg != nullptr) {
      // Check left or right image.
      if (msg_topic == depth_imgs_topic_) {
        rosbag_data_->depth_imgs_.push_back(img_msg);
      } else if (msg_topic == semantic_imgs_topic_) {
        rosbag_data_->semantic_imgs_.push_back(img_msg);
      } else if (msg_topic == rgb_imgs_topic_) {
        rosbag_data_->rgb_imgs_.push_back(img_msg);
      } else {
        LOG(WARNING) << "Img with unexpected topic: " << msg_topic;
      }
      if (kEarlyStopForDebug &&
          rosbag_data_->depth_imgs_.size() ==
              rosbag_data_->semantic_imgs_.size() &&
          rosbag_data_->depth_imgs_.size() ==
              rosbag_data_->rgb_imgs_.size() &&
          rosbag_data_->depth_imgs_.size() >= 300u) {
        LOG(ERROR) << "Early break.";
        break;
      }
      continue;
    }

    tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != nullptr) {
      if (msg_topic == "/tf") {
        LOG_EVERY_N(ERROR, 1000) << "GOT TF";
      } else {
        CHECK_EQ(msg_topic, "/tf_static");
      }
      for (const geometry_msgs::TransformStamped& transform_stamped :
           tf_msg->transforms) {
        LOG_EVERY_N(ERROR, 1000) << "Storing TFs";
        // It may be the case that we only have base_link to world_frame,
        // and then we need to apply tf from base_link to depth_cam...
        tf::StampedTransform tf;
        tf::transformStampedMsgToTF(transform_stamped, tf);
        if (transform_stamped.child_frame_id == sensor_frame_id_ &&
            transform_stamped.header.frame_id == base_link_gt_frame_id_) {
          rosbag_data_->camera_to_base_link_tf_static_ = tf;
        } else {
          rosbag_data_->tf_listener_.setTransform(tf);
        }
      }
      continue;
    }

    sensor_msgs::CameraInfoConstPtr cam_info_msg =
        msg.instantiate<sensor_msgs::CameraInfo>();
    if (cam_info_msg != nullptr) {
      if (msg_topic == left_cam_info_topic_) {
        rosbag_data_->cam_info_ = cam_info_msg;
      }
      continue;
    } else {
      LOG(ERROR) << "Could not find the type of this rosbag msg from topic:\n"
                 << msg_topic;
    }
  }
  bag.close();

  // Sanity check:
  LOG_IF(FATAL, rosbag_data_->depth_imgs_.size() == 0)
      << "No depth images parsed from rosbag.";
  LOG_IF(FATAL, rosbag_data_->semantic_imgs_.size() == 0)
      << "No semantic images  parsed from rosbag.";
  LOG_IF(FATAL, rosbag_data_->rgb_imgs_.size() == 0)
      << "No rgb images  parsed from rosbag.";
  LOG_IF(FATAL,
         rosbag_data_->depth_imgs_.size() != rosbag_data_->semantic_imgs_.size())
      << "Unequal number of depth and semantic images.";
  LOG_IF(FATAL,
         rosbag_data_->depth_imgs_.size() != rosbag_data_->rgb_imgs_.size())
      << "Unequal number of depth and rgb images.";
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
  if (k_last_kf_ < rosbag_data_->depth_imgs_.size() &&
      k_last_kf_ < rosbag_data_->semantic_imgs_.size() &&
      k_last_kf_ < rosbag_data_->rgb_imgs_.size()) {
    while (timestamp_last_kf_ < timestamp_kf &&
           k_last_kf_ < rosbag_data_->depth_imgs_.size() &&
           k_last_kf_ < rosbag_data_->semantic_imgs_.size() &&
           k_last_kf_ < rosbag_data_->rgb_imgs_.size()) {
      depth_img_pub_.publish(rosbag_data_->depth_imgs_.at(k_last_kf_));
      semantic_img_pub_.publish(rosbag_data_->semantic_imgs_.at(k_last_kf_));
      rgb_img_pub_.publish(rosbag_data_->rgb_imgs_.at(k_last_kf_));
      timestamp_last_kf_ =
          rosbag_data_->depth_imgs_.at(k_last_kf_)->header.stamp;
      k_last_kf_++;
    }
  }
}

}  // namespace kimera
