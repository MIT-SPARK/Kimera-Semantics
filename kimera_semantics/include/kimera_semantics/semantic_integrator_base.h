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

#include <memory>
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

class SemanticIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<SemanticIntegratorBase> Ptr;
  typedef vxb::
      ApproxHashArray<12, std::mutex, vxb::GlobalIndex, vxb::LongIndexHash>
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

    std::shared_ptr<SemanticLabel2Color> semantic_label_to_color_ = nullptr;

    /// Semantic labels for dynamic objects that are not supposed to be
    /// integrated in the voxel grid.
    SemanticLabels dynamic_labels_ = SemanticLabels();
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

 protected:
  /// Thread safe.
  inline bool isSemanticLabelValid(const SemanticLabel& semantic_label) const {
    // We discard any point in the dynamic semantic labels.
    return std::find(semantic_config_.dynamic_labels_.begin(),
                     semantic_config_.dynamic_labels_.end(),
                     semantic_label) == semantic_config_.dynamic_labels_.end();
  }

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

}  // Namespace kimera
