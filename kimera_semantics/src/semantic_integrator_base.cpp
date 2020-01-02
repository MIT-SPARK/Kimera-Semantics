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
 * @file   semantic_tsdf_integrator.cpp
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#include "kimera_semantics/semantic_integrator_base.h"

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

SemanticIntegratorBase::SemanticIntegratorBase(
    const SemanticConfig& semantic_config,
    vxb::Layer<SemanticVoxel>* semantic_layer)
    : semantic_config_(semantic_config),
      semantic_layer_(nullptr),
      temp_semantic_block_mutex_(),
      temp_semantic_block_map_(),
      log_match_probability_(),
      log_non_match_probability_(),
      semantic_log_likelihood_(),
      semantic_voxel_size_(),
      semantic_voxels_per_side_(),
      semantic_block_size_(),
      semantic_voxel_size_inv_(),
      semantic_voxels_per_side_inv_(),
      semantic_block_size_inv_() {
  setSemanticLayer(semantic_layer);
  CHECK_NOTNULL(semantic_layer_);
  setSemanticProbabilities();
}

void SemanticIntegratorBase::setSemanticLayer(
    vxb::Layer<SemanticVoxel>* semantic_layer) {
  CHECK_NOTNULL(semantic_layer);

  semantic_layer_ = semantic_layer;

  semantic_voxel_size_ = semantic_layer_->voxel_size();
  semantic_block_size_ = semantic_layer_->block_size();
  semantic_voxels_per_side_ = semantic_layer_->voxels_per_side();

  semantic_voxel_size_inv_ = 1.0 / semantic_voxel_size_;
  semantic_block_size_inv_ = 1.0 / semantic_block_size_;
  semantic_voxels_per_side_inv_ = 1.0 / semantic_voxels_per_side_;
}

void SemanticIntegratorBase::setSemanticProbabilities() {
  SemanticProbability match_probability =
      semantic_config_.semantic_measurement_probability_;
  SemanticProbability non_match_probability =
      1.0f - semantic_config_.semantic_measurement_probability_;
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

// TODO(Toni): Complete this function!!
SemanticProbability SemanticIntegratorBase::computeMeasurementProbability(
    vxb::FloatingPoint ray_distance) {
  return 1.0;
}

void SemanticIntegratorBase::updateSemanticVoxel(
    const vxb::GlobalIndex& global_voxel_idx,
    const SemanticProbabilities& measurement_frequencies,
    Mutexes* mutexes,
    vxb::TsdfVoxel* tsdf_voxel,
    SemanticVoxel* semantic_voxel) {
  DCHECK(mutexes != nullptr);
  DCHECK(tsdf_voxel != nullptr);
  DCHECK(semantic_voxel != nullptr);

  // TODO(Toni): ideally, only lock once in updateTsdfVoxel, but we need to
  // modify Voxblox for that.
  // Lookup the mutex that is responsible for this voxel and lock it
  std::lock_guard<std::mutex> lock(mutexes->get(global_voxel_idx));

  // TODO(Toni): ideally, return new_sdf from updateTsdfVoxel, but we need to
  // modify Voxblox for that.
  // Similar to color blending in Voxblox tsdf_integrator.cc,
  // Quoting Voxblox:
  // color blending is expensive only do it close to the surface
  //  if (std::abs(sdf) < config_.default_truncation_distance) {
  // Do the same for semantic updates:
  // Don't do it bcs of copyrights... :'(
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

  // Actually change the color of the TSDF voxel so we do not need to change a
  // single line for the meshing with respect to Voxblox.
  switch (semantic_config_.color_mode) {
    case ColorMode::kColor:
      // Nothing, base class colors the tsdf voxel for us.
      break;
    case ColorMode::kSemantic:
      tsdf_voxel->color = semantic_voxel->color;
      break;
    case ColorMode::kSemanticProbability:
      // TODO(Toni): Might be a bit expensive to calc all these exponentials...
      tsdf_voxel->color = vxb::rainbowColorMap(std::exp(
          semantic_voxel->semantic_priors[semantic_voxel->semantic_label]));
      break;
    default:
      LOG(FATAL) << "Unknown semantic color mode: "
                 << static_cast<std::underlying_type<ColorMode>::type>(
                        semantic_config_.color_mode);
      break;
  }
  ////////////////////////////////////////////////////////////////////////////
  //}
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
SemanticVoxel* SemanticIntegratorBase::allocateStorageAndGetSemanticVoxelPtr(
    const vxb::GlobalIndex& global_voxel_idx,
    vxb::Block<SemanticVoxel>::Ptr* last_block,
    vxb::BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const vxb::BlockIndex& block_idx = vxb::getBlockIndexFromGlobalVoxelIndex(
      global_voxel_idx, semantic_voxels_per_side_inv_);

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
              semantic_voxels_per_side_,
              semantic_voxel_size_,
              vxb::getOriginPointFromGridIndex(block_idx,
                                               semantic_block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  // Only used if someone calls the getAllUpdatedBlocks I believe.
  (*last_block)->updated() = true;

  const vxb::VoxelIndex local_voxel_idx = vxb::getLocalFromGlobalVoxelIndex(
      global_voxel_idx, semantic_voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

// NOT THREAD SAFE
void SemanticIntegratorBase::updateSemanticLayerWithStoredBlocks() {
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
void SemanticIntegratorBase::updateSemanticVoxelProbabilities(
    const SemanticProbabilities& measurement_frequencies,
    SemanticProbabilities* semantic_prior_probability) const {
  DCHECK(semantic_prior_probability != nullptr);
  DCHECK_EQ(semantic_prior_probability->size(), kTotalNumberOfLabels);
  DCHECK_LE((*semantic_prior_probability)[0], 0.0);
  DCHECK(std::isfinite((*semantic_prior_probability)[0]));
  DCHECK(!semantic_prior_probability->hasNaN());
  DCHECK_EQ(measurement_frequencies.size(), kTotalNumberOfLabels);
  DCHECK_GE(measurement_frequencies.sum(), 1.0)
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
  DCHECK(!semantic_prior_probability->hasNaN());

  // Normalize posterior probability.
  // TODO(Toni): no need to normalize all the time unless someone asks
  // for meaningful probabilities?
  // normalizeProbabilities(semantic_prior_probability);
}

// THREAD SAFE
void SemanticIntegratorBase::normalizeProbabilities(
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
void SemanticIntegratorBase::calculateMaximumLikelihoodLabel(
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
void SemanticIntegratorBase::updateSemanticVoxelColor(
    const SemanticLabel& semantic_label,
    HashableColor* semantic_voxel_color) const {
  CHECK_NOTNULL(semantic_voxel_color);
  // Do Not modify semantic_label_color_map_ here bcs we need to remain
  // thread-safe, and adding mutexes for coloring seems silly.
  // Precompute, for all possible SemanticLabels (255) a color.
  *semantic_voxel_color =
      semantic_config_.semantic_label_to_color_->getColorFromSemanticLabel(
          semantic_label);
}

}  // Namespace kimera
