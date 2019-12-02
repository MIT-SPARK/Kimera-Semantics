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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @file   semantic_tsdf_integrator.h
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Core>

#include <voxblox/integrator/tsdf_integrator.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_integrator_base.h"
#include "kimera_semantics/semantic_voxel.h"

namespace kimera {

/**
 * @class FastSemanticTsdfIntegrator
 * An integrator that prioritizes speed over everything else. Rays are cast from
 * the pointcloud to the sensor origin. If a ray intersects
 * max_consecutive_ray_collisions voxels in a row that have already been updated
 * by other rays from the same cloud, it is terminated early. This results in a
 * large reduction in the number of freespace updates and greatly improves
 * runtime while ensuring all voxels receive at least a minimum number of
 * updates. Speed is further enhanced through limiting the number of rays cast
 * from each voxel as set by start_voxel_subsampling_factor and use of the
 * ApproxHashSet. Up to an order of magnitude faster then the other integrators
 * for small voxels.
 */
class FastSemanticTsdfIntegrator : public vxb::TsdfIntegratorBase,
                                   public SemanticIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FastSemanticTsdfIntegrator(const Config& config,
                             const SemanticConfig& semantic_config,
                             vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
                             vxb::Layer<SemanticVoxel>* semantic_layer);

  virtual ~FastSemanticTsdfIntegrator() = default;

  virtual void integrateSemanticFunction(const vxb::Transformation& T_G_C,
                                         const vxb::Pointcloud& points_C,
                                         const vxb::Colors& colors,
                                         const SemanticLabels& semantic_labels,
                                         const bool freespace_points,
                                         vxb::ThreadSafeIndex* index_getter);

  virtual void integratePointCloud(
      const vxb::Transformation& T_G_C,
      const vxb::Pointcloud& points_C,
      const vxb::Colors& colors,
      const bool freespace_points = false) override;

 private:
  // Everything below is basically taken directly from Voxblox fast approach.
  /**
   * Two approximate sets are used below. The limitations of these sets are
   * outlined in approx_hash_array.h, but in brief they are thread safe and very
   * fast, but have a small chance of returning false positives and false
   * negatives. As rejecting a ray or integrating an uninformative ray are not
   * very harmful operations this trade-off works well in this integrator.
   */

  /**
   * uses 2^20 bytes (8 megabytes) of ram per tester
   * A testers false negative rate is inversely proportional to its size
   */
  static constexpr size_t masked_bits_ = 20;
  /**
   * only needs to zero the above 8mb of memory once every 10,000 scans
   * (uses an additional 80,000 bytes)
   */
  static constexpr size_t full_reset_threshold_ = 10000;

  /**
   * Voxel start locations are added to this set before ray casting. The ray
   * casting only occurs if no ray has been cast from this location for this
   * scan.
   */
  vxb::ApproxHashSet<masked_bits_,
                     full_reset_threshold_,
                     vxb::GlobalIndex,
                     vxb::LongIndexHash>
      start_voxel_approx_set_;

  /**
   * This set records which voxels a scans rays have passed through. If a ray
   * moves through max_consecutive_ray_collisions voxels in a row that have
   * already been seen this scan, it is deemed to be adding no new information
   * and the casting stops.
   */
  vxb::ApproxHashSet<masked_bits_,
                     full_reset_threshold_,
                     vxb::GlobalIndex,
                     vxb::LongIndexHash>
      voxel_observed_approx_set_;

  /// Used in terminating the integration early if it exceeds a time limit.
  std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;
};

}  // Namespace kimera
