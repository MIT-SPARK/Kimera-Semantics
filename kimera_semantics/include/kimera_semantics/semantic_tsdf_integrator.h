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

enum class ColorMode : int {
  kColor = 0,
  kSemantic = 1,
  kSemanticProbability = 2,
};

//! Implemented types of semantic integrators.
enum class SemanticTsdfIntegratorType : int {
  kMerged = 0,
  kFast = 1,
};
const std::array<std::string, 2>
kSemanticTsdfIntegratorTypeNames = {{/*kMerged*/ "merged", /*kFast*/ "fast"}};

class SemanticIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<SemanticIntegratorBase> Ptr;
  typedef vxb::ApproxHashArray<12, std::mutex, vxb::GlobalIndex, vxb::LongIndexHash>
  Mutexes;

  struct SemanticConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Likelihood probability of observing a measurement given that the prior
    // semantic label is the same as the measurement.
    // This number has to be a valid probability between 0 and 1.
    // Our current model derives the likelihood of observing a semantic label
    // for a voxel with a currently different label match as:
    // probability of non-match = 1 - measurement_probability_.
    SemanticProbability semantic_measurement_probability_ = 0.9f;

    /// How to color the semantic mesh.
    ColorMode color_mode = ColorMode::kSemantic;

    SemanticLabelToColorMap semantic_label_color_map_ =
        getRandomSemanticLabelToColorMap();

    // TODO(Toni): this is just to hack our way through the fact that our images
    // are not label ids but just colors :( This is not used if the pointclouds
    // you integrate have associated label ids. It is just for the case where
    // you use its colors as ids.
    ColorToSemanticLabelMap color_to_semantic_label_map_;
  };

  SemanticIntegratorBase(const SemanticConfig& semantic_config,
                         vxb::Layer<SemanticVoxel>* semantic_layer);

  // TODO(Toni): Complete this function!!
  SemanticProbability computeMeasurementProbability(
      vxb::FloatingPoint ray_distance);

  // SHOULD BE THREAD SAFE. Updates semantic_voxel probabilities given
  // semantic_label measurement and confidence.

  /**
   * @brief updateSemanticVoxel
   * Probabilistically updates semantic voxels according to a measurement model.
   * TODO(Toni): parametrize the measurement model.
   *
   * @param global_voxel_idx
   * @param measurement_frequencies
   * @param mutexes: we pass the list of mutexes bcs it would be too wasteful to
   * have to lock two mutexes (one for the tsdf_voxel, another for the
   * semantic_voxel, instead we expect the user to send the mutexes for
   * tsdf_voxels, and we will always lock them when updating semantic_voxels.
   * @param tsdf_voxel
   * @param semantic_voxel
   */
  void updateSemanticVoxel(const vxb::GlobalIndex& global_voxel_idx,
                           const SemanticProbabilities& measurement_frequencies,
                           Mutexes* mutexes,
                           vxb::TsdfVoxel* tsdf_voxel,
                           SemanticVoxel* semantic_voxel);

  // Will return a pointer to a voxel located at global_voxel_idx in the label
  // layer. Thread safe.
  // Takes in the last_block_idx and last_block to prevent unneeded map
  // lookups. If the block this voxel would be in has not been allocated, a
  // block in temp_label_block_map_ is created/accessed and a voxel from this
  // map is returned instead. Unlike the layer, accessing
  // temp_label_block_map_ is controlled via a mutex allowing it to grow
  // during integration. These temporary blocks can be merged into the layer
  // later by calling updateLayerWithStoredBlocks()
  SemanticVoxel* allocateStorageAndGetSemanticVoxelPtr(
      const vxb::GlobalIndex& global_voxel_idx,
      vxb::Block<SemanticVoxel>::Ptr* last_semantic_block,
      vxb::BlockIndex* last_block_idx);

  // NOT THREAD SAFE
  void updateSemanticLayerWithStoredBlocks();

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
  void updateSemanticVoxelProbabilities(
      const SemanticProbabilities& measurement_frequencies,
      SemanticProbabilities* semantic_prior_probability) const;

  // THREAD SAFE
  void normalizeProbabilities(SemanticProbabilities* unnormalized_probs) const;

  // THREAD SAFE
  inline void calculateMaximumLikelihoodLabel(
      const SemanticProbabilities& semantic_posterior,
      SemanticLabel* semantic_label) const;

  // THREAD SAFE
  void updateSemanticVoxelColor(const SemanticLabel& semantic_label,
                                HashableColor* semantic_voxel_color) const;

private:
  /**
   * @brief setSemanticLayer
   * Helper function to store members of the class related to the semantic layer
   * as well as the semantic layer itself from the provided pointer.
   * @param semantic_layer pointer to an already created semantic_layer
   */
  void setSemanticLayer(vxb::Layer<SemanticVoxel>* semantic_layer);

  /**
   * @brief setSemanticProbabilities
   * Helper function to set and store the measurement likelihood function.
   */
  void setSemanticProbabilities();

public:
  /// Configuration
  const SemanticConfig semantic_config_;

  /// Layer with semantic information.
  vxb::Layer<SemanticVoxel>* semantic_layer_;

  /**
   * Temporary block storage, used to hold blocks that need to be created while
   * integrating a new pointcloud
   */
  mutable std::mutex temp_semantic_block_mutex_;
  vxb::Layer<SemanticVoxel>::BlockHashMap temp_semantic_block_map_;

  // Log probabilities of matching measurement and prior label,
  // and non-matching.
  SemanticProbability log_match_probability_;
  SemanticProbability log_non_match_probability_;
  // A `#Labels X #Labels` Eigen matrix where each `j` column represents the
  // probability of observing label `j` when current label is `i`, where `i`
  // is the row index of the matrix.
  SemanticLikelihoodFunction semantic_log_likelihood_;

  /// We keep these in case someone wants to experiment with different
  /// semantic/tsdf voxel sizes.
  // Cached map config.
  vxb::FloatingPoint semantic_voxel_size_;
  size_t semantic_voxels_per_side_;
  vxb::FloatingPoint semantic_block_size_;

  // Derived types.
  vxb::FloatingPoint semantic_voxel_size_inv_;
  vxb::FloatingPoint semantic_voxels_per_side_inv_;
  vxb::FloatingPoint semantic_block_size_inv_;
};

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
                             vxb::Layer<SemanticVoxel>* semantic_layer)
    : TsdfIntegratorBase(config, tsdf_layer),
      SemanticIntegratorBase(semantic_config, semantic_layer) {}

  virtual ~FastSemanticTsdfIntegrator() = default;

  void integrateSemanticFunction(const vxb::Transformation& T_G_C,
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
      if (!isPointValid(point_C, freespace_points, &is_clearing)) {
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
      vxb::RayCaster ray_caster(origin, point_G, is_clearing,
                           config_.voxel_carving_enabled,
                           config_.max_ray_length_m, voxel_size_inv_,
                           config_.default_truncation_distance, cast_from_origin);

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

        SemanticVoxel* semantic_voxel =
            allocateStorageAndGetSemanticVoxelPtr(global_voxel_idx, &semantic_block, &semantic_block_idx);
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

  virtual void integratePointCloud(const vxb::Transformation& T_G_C,
                                   const vxb::Pointcloud& points_C,
                                   const vxb::Colors& colors,
                                   const bool freespace_points = false) override {
    SemanticLabels semantic_labels(colors.size());
    // TODO(Toni): parallelize with openmp
    for (size_t i = 0; i < colors.size(); i++) {
      const vxb::Color& color = colors[i];
      semantic_labels[i] =
          semantic_config_.color_to_semantic_label_map_.at(
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
            this, T_G_C, points_C, colors, semantic_labels,
            freespace_points, index_getter.get());
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

 private:
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
  vxb::ApproxHashSet<masked_bits_, full_reset_threshold_,
  vxb::GlobalIndex, vxb::LongIndexHash>
      start_voxel_approx_set_;

  /**
   * This set records which voxels a scans rays have passed through. If a ray
   * moves through max_consecutive_ray_collisions voxels in a row that have
   * already been seen this scan, it is deemed to be adding no new information
   * and the casting stops.
   */
  vxb::ApproxHashSet<masked_bits_, full_reset_threshold_,
  vxb::GlobalIndex, vxb::LongIndexHash>
      voxel_observed_approx_set_;

  /// Used in terminating the integration early if it exceeds a time limit.
  std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;

};


/// Creates a Semantic/TSDF integrator of the desired type.
class SemanticTsdfIntegratorFactory {
public:
  static std::unique_ptr<vxb::TsdfIntegratorBase> create(
      const std::string& integrator_type_name,
      const vxb::TsdfIntegratorBase::Config& config,
      const SemanticIntegratorBase::SemanticConfig& semantic_config,
      vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
      vxb::Layer<SemanticVoxel>* semantic_layer) {
    CHECK(!integrator_type_name.empty());

    int integrator_type = 0;
    for (const std::string& valid_integrator_type_name :
         kSemanticTsdfIntegratorTypeNames) {
      if (integrator_type_name == valid_integrator_type_name) {
        return create(static_cast<SemanticTsdfIntegratorType>(integrator_type),
                      config, semantic_config, tsdf_layer, semantic_layer);
      }
      ++integrator_type;
    }
    LOG(FATAL) << "Unknown TSDF integrator type: " << integrator_type_name;
    return nullptr;
  }

  static std::unique_ptr<vxb::TsdfIntegratorBase> create(
      const SemanticTsdfIntegratorType integrator_type,
      const vxb::TsdfIntegratorBase::Config& config,
      const SemanticIntegratorBase::SemanticConfig& semantic_config,
      vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
      vxb::Layer<SemanticVoxel>* semantic_layer) {
    CHECK_NOTNULL(tsdf_layer);
    switch (integrator_type) {
    case SemanticTsdfIntegratorType::kFast:
      return kimera::make_unique<FastSemanticTsdfIntegrator>(
            config, semantic_config, tsdf_layer, semantic_layer);
      break;
    case SemanticTsdfIntegratorType::kMerged:
      return kimera::make_unique<MergedSemanticTsdfIntegrator>(
            config, semantic_config, tsdf_layer, semantic_layer);
      break;
    default:
      LOG(FATAL) << "Unknown Semantic/TSDF integrator type: "
                 << static_cast<int>(integrator_type);
      break;
    }
    return nullptr;
  }
private:
  /// Factories should be singletons, or rather not instantiable.
  SemanticTsdfIntegratorFactory() = default;
  virtual ~SemanticTsdfIntegratorFactory() = default;

};

}  // Namespace kimera
