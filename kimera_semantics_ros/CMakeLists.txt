cmake_minimum_required(VERSION 2.8.3)
project(kimera_semantics_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/semantic_tsdf_server.cpp
  src/ros_params.cpp
)

cs_add_executable(kimera_semantics_node
  src/kimera_semantics_node.cpp
)
target_link_libraries(kimera_semantics_node ${PROJECT_NAME})

cs_add_executable(semantic_simulator_eval
  src/semantic_simulation_eval.cpp
)
target_link_libraries(semantic_simulator_eval ${PROJECT_NAME})

cs_install()

cs_export()
