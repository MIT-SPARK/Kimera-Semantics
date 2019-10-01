cmake_minimum_required(VERSION 2.8.3)
project(kimera_semantics_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/kimera_semantics_node.cpp
)
#target_link_libraries(${PROJECT_NAME} SparkVio)

cs_install()

cs_export()
