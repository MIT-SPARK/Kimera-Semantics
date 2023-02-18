cmake_minimum_required(VERSION 3.1)
project(kimera_semantics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  include/kimera_semantics/color.h
  include/kimera_semantics/common.h
  include/kimera_semantics/csv_iterator.h
  include/kimera_semantics/semantic_integrator_base.h
  include/kimera_semantics/semantic_tsdf_integrator_factory.h
  include/kimera_semantics/semantic_tsdf_integrator_fast.h
  include/kimera_semantics/semantic_tsdf_integrator_merged.h
  include/kimera_semantics/semantic_voxel.h
  src/color.cpp
  src/csv_iterator.cpp
  src/semantic_integrator_base.cpp
  src/semantic_tsdf_integrator_factory.cpp
  src/semantic_tsdf_integrator_fast.cpp
  src/semantic_tsdf_integrator_merged.cpp
  src/simulation/semantic_simulation_world.cpp
)

cs_install()

cs_export()
