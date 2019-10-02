cmake_minimum_required(VERSION 2.8.3)
project(kimera_semantics)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/semantic_tsdf_integrator.cpp
)

cs_install()

cs_export()
