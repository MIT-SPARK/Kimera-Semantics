// NOTE: Most code is derived from voxblox: github.com/ethz-asl/voxblox
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @file   semantic_tsdf_integrator.h
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Core>

#include <voxblox/integrator/tsdf_integrator.h>

#include "kimera_semantics/color.h"
#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_voxel.h"
#include "kimera_semantics/semantic_integrator_base.h"

namespace kimera {

/**
 * Semantic TSDF integrator.
 * Uses ray bundling to improve integration speed, points which lie in the same
 * voxel are "merged" into a single point. Raycasting and updating then proceeds
 * as normal. Fast for large voxels, with minimal loss of information.
 */
class MergedSemanticTsdfIntegrator : public vxb::MergedTsdfIntegrator,
                                     public SemanticIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef vxb::LongIndexHashMapType<vxb::AlignedVector<size_t>>::type VoxelMap;
  typedef VoxelMap::value_type VoxelMapElement;

  MergedSemanticTsdfIntegrator(const Config& config,
                         const SemanticConfig& semantic_config,
                         vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
                         vxb::Layer<SemanticVoxel>* semantic_layer);
  virtual ~MergedSemanticTsdfIntegrator() = default;

  virtual void integratePointCloud(const vxb::Transformation& T_G_C,
                                   const vxb::Pointcloud& points_C,
                                   const vxb::Colors& colors,
                                   const bool freespace_points = false) override;

  // Use if you don't have labels, but the info is encoded in colors.
  // Otw, use integratePointCloud directly with semantic labels.
  void integratePointCloud(const vxb::Transformation& T_G_C,
                           const vxb::Pointcloud& points_C,
                           const HashableColors& colors,
                           const bool freespace_points = false);

  void integratePointCloud(const vxb::Transformation& T_G_C,
                           const vxb::Pointcloud& points_C,
                           const HashableColors& colors,
                           const SemanticLabels& semantic_labels,
                           const bool freespace_points = false);

 protected:
  void integrateRays(const vxb::Transformation& T_G_C,
                     const vxb::Pointcloud& points_C,
                     const HashableColors& colors,
                     const SemanticLabels& semantic_labels,
                     const bool enable_anti_grazing,
                     const bool clearing_ray,
                     const VoxelMap& voxel_map,
                     const VoxelMap& clear_map);

  // NEEDS TO BE THREAD-SAFE
  void integrateVoxels(const vxb::Transformation& T_G_C,
                       const vxb::Pointcloud& points_C,
                       const HashableColors& colors,
                       const SemanticLabels& semantic_labels,
                       const bool enable_anti_grazing,
                       const bool clearing_ray,
                       const VoxelMap& voxel_map,
                       const VoxelMap& clear_map,
                       const size_t thread_idx);

  // HAS TO BE THREADSAFE!!!
  void integrateVoxel(const vxb::Transformation& T_G_C,
                      const vxb::Pointcloud& points_C,
                      const HashableColors& colors,
                      const SemanticLabels& semantic_labels,
                      const bool enable_anti_grazing,
                      const bool clearing_ray,
                      const VoxelMapElement& global_voxel_idx_to_point_indices,
                      const VoxelMap& voxel_map);

 private:
};

}  // Namespace kimera
