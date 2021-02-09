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

/**
 * @file   semantic_tsdf_integrator.cpp
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#include "kimera_semantics/semantic_tsdf_integrator_merged.h"

#include <list>
#include <memory>
#include <utility>

#include <glog/logging.h>

#include <Eigen/Core>

#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/timing.h>

#include "kimera_semantics/color.h"
#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_voxel.h"

namespace kimera {

MergedSemanticTsdfIntegrator::MergedSemanticTsdfIntegrator(
    const Config& config,
    const SemanticConfig& semantic_config,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
    vxb::Layer<SemanticVoxel>* semantic_layer)
    : MergedTsdfIntegrator(config, CHECK_NOTNULL(tsdf_layer)),
      SemanticIntegratorBase(semantic_config, CHECK_NOTNULL(semantic_layer)) {}

// Use if you don't have labels, but the info is encoded in colors.
// Otw, use integratePointCloud directly with semantic labels.
void MergedSemanticTsdfIntegrator::integratePointCloud(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const vxb::Colors& colors,
    const bool freespace_points) {
  HashableColors hash_colors(colors.size());
  SemanticLabels semantic_labels(colors.size());
  // TODO(Toni): parallelize with openmp
  for (size_t i = 0; i < colors.size(); i++) {
    // TODO(Toni): this will break badly if the color is not found in the
    // map, and alerting the user about that might be too costly.
    // TODO(Toni): currently simulator sends sporadic 255,255,255,255 for
    // rays traced to nowhere... map color::White() to label unknown 0u.
    // LOG(ERROR) << "RGB: " << std::to_string(colors[i].r) << ' '
    //            <<  std::to_string(colors[i].g) << ' '
    //            <<  std::to_string(colors[i].b);
    // TODO(Toni): Pointcloud recolor sets `a` field to 0. Making the
    // map lookup fail.
    const vxb::Color& color = colors[i];
    CHECK(semantic_config_.semantic_label_to_color_);
    semantic_labels[i] =
        semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(
            HashableColor(color.r, color.g, color.b, 255u));
  }

  vxb::timing::Timer integrate_pcl_semantic_tsdf_timer(
      "semantic_tsdf/integrate");
  integratePointCloud(
      T_G_C, points_C, hash_colors, semantic_labels, freespace_points);
  integrate_pcl_semantic_tsdf_timer.Stop();
}

void MergedSemanticTsdfIntegrator::integratePointCloud(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const HashableColors& colors,
    const SemanticLabels& semantic_labels,
    const bool freespace_points) {
  CHECK_GE(points_C.size(), 0u);
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), semantic_labels.size());
  vxb::timing::Timer integrate_timer("integrate/semantic_merged");

  // Pre-compute a list of unique voxels to end on.
  // Create a hashmap: VOXEL INDEX -> index in original cloud.
  VoxelMap voxel_map;
  // This is a hash map (same as above) to all the indices that need to be
  // cleared.
  VoxelMap clear_map;

  std::unique_ptr<vxb::ThreadSafeIndex> index_getter(
      vxb::ThreadSafeIndexFactory::get(config_.integration_order_mode,
                                       points_C));

  bundleRays(T_G_C,
             points_C,
             freespace_points,
             index_getter.get(),
             &voxel_map,
             &clear_map);

  integrateRays(T_G_C,
                points_C,
                colors,
                semantic_labels,
                config_.enable_anti_grazing,
                false,
                voxel_map,
                clear_map);

  vxb::timing::Timer clear_timer("integrate/clear");

  integrateRays(T_G_C,
                points_C,
                colors,
                semantic_labels,
                config_.enable_anti_grazing,
                true,
                voxel_map,
                clear_map);

  clear_timer.Stop();

  integrate_timer.Stop();
}

void MergedSemanticTsdfIntegrator::integrateRays(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const HashableColors& colors,
    const SemanticLabels& semantic_labels,
    const bool enable_anti_grazing,
    const bool clearing_ray,
    const VoxelMap& voxel_map,
    const VoxelMap& clear_map) {
  // if only 1 thread just do function call, otherwise spawn threads
  if (config_.integrator_threads == 1u) {
    constexpr size_t thread_idx = 0u;
    integrateVoxels(T_G_C,
                    points_C,
                    colors,
                    semantic_labels,
                    enable_anti_grazing,
                    clearing_ray,
                    voxel_map,
                    clear_map,
                    thread_idx);
  } else {
    std::list<std::thread> integration_threads;
    for (size_t i = 0u; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(&MergedSemanticTsdfIntegrator::integrateVoxels,
                                       this,
                                       T_G_C,
                                       points_C,
                                       colors,
                                       semantic_labels,
                                       enable_anti_grazing,
                                       clearing_ray,
                                       voxel_map,
                                       clear_map,
                                       i);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  vxb::timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  updateSemanticLayerWithStoredBlocks();
  insertion_timer.Stop();
}

// NEEDS TO BE THREAD-SAFE
void MergedSemanticTsdfIntegrator::integrateVoxels(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const HashableColors& colors,
    const SemanticLabels& semantic_labels,
    const bool enable_anti_grazing,
    const bool clearing_ray,
    const VoxelMap& voxel_map,
    const VoxelMap& clear_map,
    const size_t thread_idx) {
  VoxelMap::const_iterator it;
  size_t map_size;
  if (clearing_ray) {
    it = clear_map.begin();
    map_size = clear_map.size();
  } else {
    it = voxel_map.begin();
    map_size = voxel_map.size();
  }
  for (size_t i = 0u; i < map_size; ++i) {
    if (((i + thread_idx + 1) % config_.integrator_threads) == 0u) {
      integrateVoxel(T_G_C,
                     points_C,
                     colors,
                     semantic_labels,
                     enable_anti_grazing,
                     clearing_ray,
                     *it,
                     voxel_map);
    }
    ++it;
  }
}

// HAS TO BE THREADSAFE!!!
void MergedSemanticTsdfIntegrator::integrateVoxel(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const HashableColors& colors,
    const SemanticLabels& semantic_labels,
    const bool enable_anti_grazing,
    const bool clearing_ray,
    const VoxelMapElement& kv,
    const VoxelMap& voxel_map) {
  if (kv.second.empty()) {
    return;
  }

  const vxb::Point& origin = T_G_C.getPosition();
  HashableColor merged_color;
  vxb::Point merged_point_C = vxb::Point::Zero();
  vxb::FloatingPoint merged_weight = 0.0f;
  // Calculate semantic labels frequencies to encode likelihood function.
  // Prefill with 0 frequency.
  SemanticProbabilities semantic_label_frequencies =
      SemanticProbabilities::Zero();

  // Loop over all point indices inside current voxel.
  // Generate merged values to propagate to other voxels:
  // - merged_point_C: point in Camera frame of reference representing the
  //                   measured depth value summarized from all depth values.
  // - merged_color: color to ray-cast through the voxels.
  // - semantic_label_frequencies: number of observed labels in the voxel.
  for (const size_t& pt_idx : kv.second) {
    const vxb::Point& point_C = points_C[pt_idx];
    const HashableColor& color = colors[pt_idx];

    const vxb::FloatingPoint& point_weight = getVoxelWeight(point_C);
    if (point_weight < vxb::kEpsilon) {
      continue;
    }
    merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                     (merged_weight + point_weight);
    merged_color = HashableColor::blendTwoColors(
        merged_color, merged_weight, color, point_weight);
    merged_weight += point_weight;

    const SemanticLabel& semantic_label = semantic_labels[pt_idx];
    CHECK_LT(semantic_label, semantic_label_frequencies.size());
    semantic_label_frequencies[semantic_label] += 1.0f;

    // only take first point when clearing
    if (clearing_ray) {
      break;
    }
  }

  const vxb::Point merged_point_G = T_G_C * merged_point_C;
  vxb::RayCaster ray_caster(origin,
                            merged_point_G,
                            clearing_ray,
                            config_.voxel_carving_enabled,
                            config_.max_ray_length_m,
                            voxel_size_inv_,
                            config_.default_truncation_distance);

  vxb::GlobalIndex global_voxel_idx;
  // TODO(Toni): also put semantic_block
  // and tsdf_block outside while loop!!!!!! Why is merged not doing this???
  vxb::BlockIndex block_idx;
  vxb::BlockIndex semantic_block_idx;
  vxb::Block<vxb::TsdfVoxel>::Ptr block = nullptr;
  vxb::Block<SemanticVoxel>::Ptr semantic_block = nullptr;
  // This bool checks if the ray already updated a voxel as empty.
  bool ray_updated_voxel_empty = false;
  while (ray_caster.nextRayIndex(&global_voxel_idx)) {
    if (enable_anti_grazing) {
      // Check if this one is already the block hash map for this
      // insertion. Skip this to avoid grazing.
      if ((clearing_ray || global_voxel_idx != kv.first) &&
          voxel_map.find(global_voxel_idx) != voxel_map.end()) {
        continue;
      }
    }

    vxb::TsdfVoxel* voxel =
        allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
    updateTsdfVoxel(origin, merged_point_G, global_voxel_idx,
                    merged_color,
                    merged_weight, voxel);

    SemanticVoxel* semantic_voxel =
        allocateStorageAndGetSemanticVoxelPtr(global_voxel_idx, &semantic_block, &semantic_block_idx);
    updateSemanticVoxel(global_voxel_idx,
                        semantic_label_frequencies,
                        &mutexes_,
                        voxel,
                        semantic_voxel);
  }
}

}  // Namespace kimera
