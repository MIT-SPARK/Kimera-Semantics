cmake_minimum_required(VERSION 2.8.3)
project(kimera_semantics_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  include/kimera_semantics_ros/depth_map_to_pointcloud.h
  include/kimera_semantics_ros/ros_params.h
  include/kimera_semantics_ros/rosbag_data_provider.h
  include/kimera_semantics_ros/semantic_simulation_server.h
  include/kimera_semantics_ros/semantic_tsdf_server.h
  src/ros_params.cpp
  src/semantic_simulation_server.cpp
  src/semantic_tsdf_server.cpp
  src/rosbag_data_provider.cpp
  src/semantic_simulation_server.cpp
  src/semantic_tsdf_server.cpp
)

cs_add_executable(kimera_semantics_node
  src/kimera_semantics_node.cpp
)
target_link_libraries(kimera_semantics_node ${PROJECT_NAME})

cs_add_executable(kimera_semantics_rosbag
  src/kimera_semantics_rosbag.cpp
)
target_link_libraries(kimera_semantics_rosbag ${PROJECT_NAME})

cs_add_executable(semantic_simulator_eval
  src/semantic_simulation_eval.cpp
)
target_link_libraries(semantic_simulator_eval ${PROJECT_NAME} ${PCL_LIBRARIES})

cs_install()

cs_export()
