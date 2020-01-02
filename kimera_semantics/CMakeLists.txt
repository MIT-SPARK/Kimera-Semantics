cmake_minimum_required(VERSION 2.8.3)
project(kimera_semantics)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/color.cpp
  src/csv_iterator.cpp
  src/semantic_integrator_base.cpp
  src/semantic_tsdf_integrator_factory.cpp
  src/semantic_tsdf_integrator_fast.cpp
  src/semantic_tsdf_integrator_merged.cpp
)

cs_install()

cs_export()
