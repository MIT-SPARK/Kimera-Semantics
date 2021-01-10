#include <ros/ros.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics_ros/semantic_simulation_server.h"

namespace kimera {
class SemanticSimulationServerImpl : public SemanticSimulationServer {
 public:
  SemanticSimulationServerImpl(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
      : SemanticSimulationServer(nh, nh_private) {}

  void prepareWorld() {
    CHECK(world_);
    world_->addObject(make_unique<vxb::Sphere>(
        vxb::Point(0.0, 0.0, 2.0), 2.0, vxb::Color::Red()));

    world_->addObject(make_unique<vxb::PlaneObject>(
        vxb::Point(-2.0, -4.0, 2.0), vxb::Point(0, 1, 0), vxb::Color::White()));

    world_->addObject(make_unique<vxb::PlaneObject>(
        vxb::Point(4.0, 0.0, 0.0), vxb::Point(-1, 0, 0), vxb::Color::Pink()));

    world_->addObject(make_unique<vxb::Cube>(
        vxb::Point(-4.0, 4.0, 2.0), vxb::Point(4, 4, 4), vxb::Color::Green()));

    world_->addGroundLevel(0.03);

    world_->generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_->generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }
};

}  // namespace kimera

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_semantics_simulator");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::SemanticSimulationServerImpl sim_eval(nh, nh_private);

  sim_eval.run();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}
