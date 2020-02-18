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

/**
 * @file   semantic_tsdf_integrator_fast.cpp
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#include "kimera_semantics/semantic_tsdf_integrator_fast.h"

#include <list>
#include <memory>
#include <utility>

#include <voxblox/utils/timing.h>

#include "kimera_semantics/color.h"

namespace kimera {

FastSemanticTsdfIntegrator::FastSemanticTsdfIntegrator(
    const Config& config,
    const SemanticConfig& semantic_config,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
    vxb::Layer<SemanticVoxel>* semantic_layer)
    : TsdfIntegratorBase(config, tsdf_layer),
      SemanticIntegratorBase(semantic_config, semantic_layer) {}

void FastSemanticTsdfIntegrator::integrateSemanticFunction(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const vxb::Colors& colors,
    const SemanticLabels& semantic_labels,
    const bool freespace_points,
    vxb::ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx) &&
         (std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - integration_start_time_)
              .count() < config_.max_integration_time_s * 1000000)) {
    const vxb::Point& point_C = points_C[point_idx];
    const vxb::Color& color = colors[point_idx];
    const SemanticLabel& semantic_label = semantic_labels[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing) ||
        !isSemanticLabelValid(semantic_label)) {
      continue;
    }

    const vxb::Point origin = T_G_C.getPosition();
    const vxb::Point point_G = T_G_C * point_C;
    // Checks to see if another ray in this scan has already started 'close'
    // to this location. If it has then we skip ray casting this point. We
    // measure if a start location is 'close' to another points by inserting
    // the point into a set of voxels. This voxel set has a resolution
    // start_voxel_subsampling_factor times higher then the voxel size.
    vxb::GlobalIndex global_voxel_idx;
    global_voxel_idx = vxb::getGridIndexFromPoint<vxb::GlobalIndex>(
        point_G, config_.start_voxel_subsampling_factor * voxel_size_inv_);
    if (!start_voxel_approx_set_.replaceHash(global_voxel_idx)) {
      continue;
    }

    static constexpr bool cast_from_origin = false;
    vxb::RayCaster ray_caster(origin,
                              point_G,
                              is_clearing,
                              config_.voxel_carving_enabled,
                              config_.max_ray_length_m,
                              voxel_size_inv_,
                              config_.default_truncation_distance,
                              cast_from_origin);

    int64_t consecutive_ray_collisions = 0;

    vxb::Block<vxb::TsdfVoxel>::Ptr block = nullptr;
    vxb::BlockIndex block_idx;
    vxb::Block<SemanticVoxel>::Ptr semantic_block = nullptr;
    vxb::BlockIndex semantic_block_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      // Check if the current voxel has been seen by any ray cast this scan.
      // If it has increment the consecutive_ray_collisions counter, otherwise
      // reset it. If the counter reaches a threshold we stop casting as the
      // ray is deemed to be contributing too little new information.
      if (!voxel_observed_approx_set_.replaceHash(global_voxel_idx)) {
        ++consecutive_ray_collisions;
      } else {
        consecutive_ray_collisions = 0;
      }
      if (consecutive_ray_collisions > config_.max_consecutive_ray_collisions) {
        break;
      }

      vxb::TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
      const float weight = getVoxelWeight(point_C);

      updateTsdfVoxel(origin, point_G, global_voxel_idx, color, weight, voxel);

      SemanticVoxel* semantic_voxel = allocateStorageAndGetSemanticVoxelPtr(
          global_voxel_idx, &semantic_block, &semantic_block_idx);
      SemanticProbabilities semantic_label_frequencies =
          SemanticProbabilities::Zero();
      CHECK_LT(semantic_label, semantic_label_frequencies.size());
      semantic_label_frequencies[semantic_label] += 1.0f;
      updateSemanticVoxel(global_voxel_idx,
                          semantic_label_frequencies,
                          &mutexes_,
                          voxel,
                          semantic_voxel);
    }
  }
}

void FastSemanticTsdfIntegrator::integratePointCloud(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const vxb::Colors& colors,
    const bool freespace_points) {
  SemanticLabels semantic_labels(colors.size());
  // TODO(Toni): parallelize with openmp
  for (size_t i = 0; i < colors.size(); i++) {
    const vxb::Color& color = colors[i];
    CHECK(semantic_config_.semantic_label_to_color_);
    semantic_labels[i] =
        semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(
            HashableColor(color.r, color.g, color.b, 255u));
  }

  vxb::timing::Timer integrate_timer("integrate/fast");
  CHECK_EQ(points_C.size(), colors.size());

  integration_start_time_ = std::chrono::steady_clock::now();

  static int64_t reset_counter = 0;
  if ((++reset_counter) >= config_.clear_checks_every_n_frames) {
    reset_counter = 0;
    start_voxel_approx_set_.resetApproxSet();
    voxel_observed_approx_set_.resetApproxSet();
  }

  std::unique_ptr<vxb::ThreadSafeIndex> index_getter(
      vxb::ThreadSafeIndexFactory::get(config_.integration_order_mode,
                                       points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &FastSemanticTsdfIntegrator::integrateSemanticFunction,
        this,
        T_G_C,
        points_C,
        colors,
        semantic_labels,
        freespace_points,
        index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  integrate_timer.Stop();

  vxb::timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  updateSemanticLayerWithStoredBlocks();
  insertion_timer.Stop();
}

}  // Namespace kimera
