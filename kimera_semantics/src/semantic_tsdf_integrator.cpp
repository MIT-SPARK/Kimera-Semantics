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

#include "kimera_semantics/semantic_tsdf_integrator.h"

#include <functional>  // For placeholders for Semantics
#include <list>
#include <memory>
#include <numeric>  // For accumulate for Semantics
#include <string>
#include <utility>

#include <Eigen/Core>

#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/timing.h>

#include "kimera_semantics/color.h"
#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_voxel.h"

namespace kimera {

enum class ColorMode {
  kColor = 0,
  kNormals = 1,
  kSemantic = 2,
  kSemanticProbability = 3,
};

SemanticTsdfIntegrator::SemanticTsdfIntegrator(
    const Config& config,
    const SemanticConfig& semantic_config,
    vxb::Layer<SemanticVoxel>* semantic_layer,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer)
    : MergedTsdfIntegrator(config, CHECK_NOTNULL(tsdf_layer)),
      semantic_config_(semantic_config),
      semantic_log_likelihood_(),
      semantic_layer_(CHECK_NOTNULL(semantic_layer)) {
  SemanticProbability match_probability =
      semantic_config.semantic_measurement_probability_;
  SemanticProbability non_match_probability =
      1.0f - semantic_config.semantic_measurement_probability_;
  CHECK_GT(match_probability, 0.0);
  CHECK_GT(non_match_probability, 0.0);
  CHECK_LT(match_probability, 1.0);
  CHECK_LT(non_match_probability, 1.0);
  log_match_probability_ = std::log(match_probability);
  log_non_match_probability_ = std::log(non_match_probability);
  CHECK_GT(log_match_probability_, log_non_match_probability_)
      << "Your probabilities do not make sense... The likelihood of a "
         "label, knowing that we have measured that label, should not be"
         "smaller than the likelihood of seeing another label!";
  semantic_log_likelihood_ =
      semantic_log_likelihood_.Constant(log_non_match_probability_);
  semantic_log_likelihood_.diagonal() =
      semantic_log_likelihood_.diagonal().Constant(log_match_probability_);
  // TODO(Toni): sanity checks, set as DCHECK_EQ.
  CHECK_NEAR(semantic_log_likelihood_.diagonal().sum(),
             kTotalNumberOfLabels * log_match_probability_,
             100 * vxb::kFloatEpsilon);
  CHECK_NEAR(
      semantic_log_likelihood_.sum(),
      kTotalNumberOfLabels * log_match_probability_ +
          std::pow(kTotalNumberOfLabels, 2) * log_non_match_probability_ -
          kTotalNumberOfLabels * log_non_match_probability_,
      1000 * vxb::kFloatEpsilon);
}

// Use if you don't have labels, but the info is encoded in colors.
// Otw, use integratePointCloud directly with semantic labels.
void SemanticTsdfIntegrator::integratePointCloud(
    const vxb::Transformation& T_G_C,
    const vxb::Pointcloud& points_C,
    const vxb::Colors& colors,
    const bool freespace_points) {
  LOG(ERROR) << "INTEGRATE POINTCLOUD DERIVED";
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
    hash_colors.at(i) = HashableColor(color.r, color.g, color.b, 255u);
    // const auto& it =
    // semantic_config_.color_to_semantic_label_map_.find(color_a); if (it !=
    // semantic_config_.color_to_semantic_label_map_.end()) {
    semantic_labels[i] =
        semantic_config_.color_to_semantic_label_map_.at(
          hash_colors.at(i));
    //} else {
    //  LOG(ERROR) << "Caught an unknown color: \n"
    //             << "RGB: " << std::to_string(color_a.r) << ' '
    //             <<  std::to_string(color_a.g) << ' '
    //              <<  std::to_string(color_a.b) << ' '
    //              <<  std::to_string(color_a.a);
    //  semantic_labels[i] = 0u; // Assign unknown label for now...
    //}
  }

  vxb::timing::Timer integrate_pcl_semantic_tsdf_timer(
      "semantic_tsdf/integrate");
  integratePointCloud(
      T_G_C, points_C, hash_colors, semantic_labels, freespace_points);
  integrate_pcl_semantic_tsdf_timer.Stop();
}

void SemanticTsdfIntegrator::integratePointCloud(
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

void SemanticTsdfIntegrator::integrateRays(
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
      integration_threads.emplace_back(&SemanticTsdfIntegrator::integrateVoxels,
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
void SemanticTsdfIntegrator::integrateVoxels(
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
void SemanticTsdfIntegrator::integrateVoxel(
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
                        voxel,
                        semantic_voxel);

  }
}

// TODO(Toni): Complete this function!!
SemanticProbability SemanticTsdfIntegrator::computeMeasurementProbability(
    vxb::FloatingPoint ray_distance) {
  return 1.0;
}

void SemanticTsdfIntegrator::updateSemanticVoxel(
    const vxb::GlobalIndex& global_voxel_idx,
    const SemanticProbabilities& measurement_frequencies,
    vxb::TsdfVoxel* tsdf_voxel,
    SemanticVoxel* semantic_voxel) {
  DCHECK(tsdf_voxel != nullptr);
  DCHECK(semantic_voxel != nullptr);

  // Similar to color blending in Voxblox tsdf_integrator.cc,
  // we perform semantic inference only if close to the surface:
  // Quoting Voxblox:
  // color blending is expensive only do it close to the surface
  //  if (std::abs(sdf) < config_.default_truncation_distance) {
  if (std::abs(tsdf_voxel->distance) < config_.default_truncation_distance) {
    // Lookup the mutex that is responsible for this voxel and lock it
    std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));
    ////// Here is the actual logic of Kimera-Semantics:   /////////////////////
    // Calculate new probabilities given the measurement frequencies.
    updateSemanticVoxelProbabilities(measurement_frequencies,
                                     &semantic_voxel->semantic_priors);

    // Get MLE semantic label.
    calculateMaximumLikelihoodLabel(semantic_voxel->semantic_priors,
                                    &semantic_voxel->semantic_label);

    // Colorize according to current MLE semantic label.
    updateSemanticVoxelColor(semantic_voxel->semantic_label,
                             &semantic_voxel->color);

    // Actually hack the color of the TSDF voxel so we do not need to change a
    // single line for the meshing with respect to Voxblox.
    static const ColorMode color_mode = ColorMode::kSemanticProbability;
    switch (color_mode) {
    case ColorMode::kSemantic:
      tsdf_voxel->color = semantic_voxel->color;
      break;
    case ColorMode::kSemanticProbability:
      // TODO(Toni): Might be a bit expensive to calc all these exponentials...
      tsdf_voxel->color = vxb::rainbowColorMap(std::exp(
                                              semantic_voxel->semantic_priors[
                                              semantic_voxel->semantic_label]));
      break;
    default:
      LOG(ERROR) << "Error :*)";
      break;
    }
    ////////////////////////////////////////////////////////////////////////////
  }
}

// Will return a pointer to a voxel located at global_voxel_idx in the tsdf
// layer. Thread safe.
// Takes in the last_block_idx and last_block to prevent unneeded map lookups.
// If the block this voxel would be in has not been allocated, a block in
// temp_block_map_ is created/accessed and a voxel from this map is returned
// instead. Unlike the layer, accessing temp_block_map_ is controlled via a
// mutex allowing it to grow during integration.
// These temporary blocks can be merged into the layer later by calling
// updateLayerWithStoredBlocks()
SemanticVoxel* SemanticTsdfIntegrator::allocateStorageAndGetSemanticVoxelPtr(
    const vxb::GlobalIndex& global_voxel_idx,
    vxb::Block<SemanticVoxel>::Ptr* last_block,
    vxb::BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const vxb::BlockIndex& block_idx = vxb::getBlockIndexFromGlobalVoxelIndex(
      global_voxel_idx, voxels_per_side_inv_);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {
    *last_block = semantic_layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  if (*last_block == nullptr) {
    // To allow temp_label_block_map_ to grow we can only let
    // one thread in at once
    std::lock_guard<std::mutex> lock(temp_semantic_block_mutex_);

    typename vxb::Layer<SemanticVoxel>::BlockHashMap::iterator it =
        temp_semantic_block_map_.find(block_idx);
    if (it != temp_semantic_block_map_.end()) {
      *last_block = it->second;
    } else {
      auto insert_status = temp_semantic_block_map_.emplace(
          block_idx,
          std::make_shared<vxb::Block<SemanticVoxel>>(
              voxels_per_side_, voxel_size_,
              vxb::getOriginPointFromGridIndex(block_idx, block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  // Only used if someone calls the getAllUpdatedBlocks I believe.
  (*last_block)->updated() = true;

  const vxb::VoxelIndex local_voxel_idx =
      vxb::getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

// NOT THREAD SAFE
void SemanticTsdfIntegrator::updateSemanticLayerWithStoredBlocks() {
  vxb::BlockIndex last_block_idx;
  vxb::Block<SemanticVoxel>::Ptr block = nullptr;
  for (const std::pair<const vxb::BlockIndex, vxb::Block<SemanticVoxel>::Ptr>&
           tmp_label_block_pair : temp_semantic_block_map_) {
    semantic_layer_->insertBlock(tmp_label_block_pair);
  }
  temp_semantic_block_map_.clear();
}

/** THREAD SAFE
 * Given a measured semantic label (semantic_label),
 * a measurement probability (measurement_probability), and
 * the set of prior semantic probabilities (semantic_prior_probability).
 * Returns: __normalized__ posterior probabilities given by the update eq.:
 *
 * posterior_i = measurement_probability * prior_i; // Iff i == measured_label
 * posterior_i = (1 - measurement_probability) * prior_i; // i !=
 *mesured_label
 *
 * Typically, one would set measurement_probability to be 0.9
 * Unless prior knowledge of the likelihood of the measurement is given.
 * For example, one may encode higher probability of measuring label floor
 * if voxel is close to ground.
 **/
// TODO(Toni): Unit Test this function!
void SemanticTsdfIntegrator::updateSemanticVoxelProbabilities(
    const SemanticProbabilities& measurement_frequencies,
    SemanticProbabilities* semantic_prior_probability) const {
  CHECK_NOTNULL(semantic_prior_probability);
  CHECK_EQ(semantic_prior_probability->size(), kTotalNumberOfLabels);
  CHECK_LE((*semantic_prior_probability)[0], 0.0);
  CHECK(std::isfinite((*semantic_prior_probability)[0]));
  CHECK(!semantic_prior_probability->hasNaN());
  CHECK_EQ(measurement_frequencies.size(), kTotalNumberOfLabels);
  CHECK_GE(measurement_frequencies.sum(), 1.0)
      << "We should at least have one measurement when calling this "
         "function.";

  // Check prior is normalized!
  // static constexpr bool kDebug = true;
  // if (kDebug) {
  //  CHECK_NEAR(semantic_prior_probability->sum(), 1.0, vxb::kFloatEpsilon);
  //}

  // A. Pre-multiply each column of
  // likelihood matrix with corresponding measurement frequency.
  // B. Compute posterior probabilities:
  // Post_i = (likelihood * meas freqs)_i + prior_i;
  *semantic_prior_probability +=
      semantic_log_likelihood_ * measurement_frequencies;
  CHECK(!semantic_prior_probability->hasNaN());

  // Normalize posterior probability.
  // TODO(Toni): no need to normalize all the time unless someone asks
  // for meaningful probabilities?
  // normalizeProbabilities(semantic_prior_probability);
}

// THREAD SAFE
void SemanticTsdfIntegrator::normalizeProbabilities(
    SemanticProbabilities* unnormalized_probs) const {
  CHECK_NOTNULL(unnormalized_probs);
  CHECK_GT(unnormalized_probs->size(), 0u);
  CHECK_LT((*unnormalized_probs)[0], 0.0) << "Are you sure you are using"
                                             "log odds?";

  // Find norm
  // Make it a double so you make sure there is no overflow??
  SemanticProbability normalization_factor = unnormalized_probs->norm();
  // Eigen's normalize :
  // "If the input vector is too small (i.e., this->norm()==0),
  // then this function returns a copy of the input."
  CHECK_GE(normalization_factor, 0.0);
  if (normalization_factor != 0.0) {
    unnormalized_probs->normalize();
  } else {
    CHECK_EQ(unnormalized_probs->size(), kTotalNumberOfLabels);
    static const SemanticProbability kUniformLogProbability =
        std::log(1 / kTotalNumberOfLabels);
    LOG(WARNING) << "Normalization Factor is " << normalization_factor
                 << ", all values are 0. Normalizing to log(1/n) = "
                 << kUniformLogProbability;
    unnormalized_probs->setConstant(kUniformLogProbability);
  }

  // TODO(Toni): REMOVE
  // Check that normalization really happened.
  static constexpr bool kDebug = true;
  if (kDebug) {
    CHECK_NEAR(unnormalized_probs->norm(), 1.0f, vxb::kFloatEpsilon);
  }
}

// THREAD SAFE
void SemanticTsdfIntegrator::calculateMaximumLikelihoodLabel(
    const SemanticProbabilities& semantic_posterior,
    SemanticLabel* semantic_label) const {
  CHECK_NOTNULL(semantic_label);
  // Return semantic label with current max probability.
  // TODO(Toni): what if there is a draw?
  // Warning:
  //    the result is undefined if semantic_posterior contains NaN.
  CHECK(!semantic_posterior.hasNaN())
      << "Eigen's maxCoeff has undefined behaviour with NaNs, fix your "
         "posteriors.";
  semantic_posterior.maxCoeff(semantic_label);
}

// THREAD SAFE
void SemanticTsdfIntegrator::updateSemanticVoxelColor(
    const SemanticLabel& semantic_label,
    HashableColor* semantic_voxel_color) const {
  CHECK_NOTNULL(semantic_voxel_color);
  // Do Not modify semantic_label_color_map_ here bcs we need to remain
  // thread-safe, and adding mutexes for coloring seems silly.
  // Precompute, for all possible SemanticLabels (255) a color.
  *semantic_voxel_color =
      semantic_config_.semantic_label_color_map_.at(semantic_label);
}

}  // Namespace kimera
