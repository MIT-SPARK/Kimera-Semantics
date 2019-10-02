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

/**
 * Semantic TSDF integrator.
 * Uses ray bundling to improve integration speed, points which lie in the same
 * voxel are "merged" into a single point. Raycasting and updating then proceeds
 * as normal. Fast for large voxels, with minimal loss of information.
 */
class SemanticTsdfIntegrator : public vxb::MergedTsdfIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef vxb::LongIndexHashMapType<vxb::AlignedVector<size_t>>::type VoxelMap;
  typedef VoxelMap::value_type VoxelMapElement;

  struct SemanticConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Factor which we multiply the default_truncation_distance of
    // the TSDF layer.
    vxb::FloatingPoint semantic_truncation_distance_factor_ = 1.0;

    // Likelihood probability of observing a measurement given that the prior
    // semantic label is the same as the measurement.
    // This number has to be a valid probability between 0 and 1.
    // Our current model derives the likelihood of observing a semantic label
    // for a voxel with a currently different label match as:
    // probability of non-match = 1 - measurement_probability_.
    SemanticProbability semantic_measurement_probability_ = 0.9;

    SemanticLabelToColorMap semantic_label_color_map_ =
        getRandomSemanticLabelToColorMap();

    // TODO(Toni): this is just to hack our way through the fact that our images
    // are not label ids but just colors :( This is not used if the pointclouds
    // you integrate have associated label ids. It is just for the case where
    // you use its colors as ids.
    ColorToSemanticLabelMap color_to_semantic_label_map_;

    std::string print() const {
      std::stringstream ss;
      // clang-format off
      ss << "================== Semantic Integrator Config ====================\n";
      ss << " - semantic_truncation_distance_factor:               " << semantic_truncation_distance_factor_ << "\n";
      ss << "==============================================================\n";
      // clang-format on
      return ss.str();
    }
  };

  SemanticTsdfIntegrator(const Config& config,
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
        1.0 - semantic_config.semantic_measurement_probability_;
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
  void integratePointCloud(const vxb::Transformation& T_G_C,
                           const vxb::Pointcloud& points_C,
                           const HashableColors& colors,
                           const bool freespace_points = false) {
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
      const HashableColor& color = colors[i];
      HashableColor color_a = HashableColor(color.r, color.g, color.b, 255u);
      // const auto& it =
      // semantic_config_.color_to_semantic_label_map_.find(color_a); if (it !=
      // semantic_config_.color_to_semantic_label_map_.end()) {
      semantic_labels[i] =
          semantic_config_.color_to_semantic_label_map_.at(color_a);
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
        T_G_C, points_C, colors, semantic_labels, freespace_points);
    integrate_pcl_semantic_tsdf_timer.Stop();
  }

  void integratePointCloud(const vxb::Transformation& T_G_C,
                           const vxb::Pointcloud& points_C,
                           const HashableColors& colors,
                           const SemanticLabels& semantic_labels,
                           const bool freespace_points = false) {
    CHECK_GE(points_C.size(), 0u);
    CHECK_EQ(points_C.size(), colors.size());
    CHECK_EQ(points_C.size(), semantic_labels.size());

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

    integrateRays(T_G_C,
                  points_C,
                  colors,
                  semantic_labels,
                  config_.enable_anti_grazing,
                  true,
                  voxel_map,
                  clear_map);
  }

 protected:
  void integrateRays(const vxb::Transformation& T_G_C,
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
        integration_threads.emplace_back(
            &SemanticTsdfIntegrator::integrateVoxels,
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
  void integrateVoxels(const vxb::Transformation& T_G_C,
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
  void integrateVoxel(const vxb::Transformation& T_G_C,
                      const vxb::Pointcloud& points_C,
                      const HashableColors& colors,
                      const SemanticLabels& semantic_labels,
                      const bool enable_anti_grazing,
                      const bool clearing_ray,
                      const VoxelMapElement& global_voxel_idx_to_point_indices,
                      const VoxelMap& voxel_map) {
    if (global_voxel_idx_to_point_indices.second.empty()) {
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
    for (const size_t& pt_idx : global_voxel_idx_to_point_indices.second) {
      const vxb::Point& point_C = points_C[pt_idx];
      const HashableColor& color = colors[pt_idx];

      const vxb::FloatingPoint& point_weight = getVoxelWeight(point_C);
      // TODO(Toni) skip?
      // if (point_weight < kEpsilon) {
      //   continue;
      // }
      merged_point_C =
          (merged_point_C * merged_weight + point_C * point_weight) /
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
    vxb::BlockIndex tsdf_block_idx;
    vxb::BlockIndex semantic_block_idx;
    vxb::Block<vxb::TsdfVoxel>::Ptr tsdf_block = nullptr;
    vxb::Block<SemanticVoxel>::Ptr semantic_block = nullptr;
    // This bool checks if the ray already updated a voxel as empty.
    bool ray_updated_voxel_empty = false;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      if (enable_anti_grazing) {
        // Check if this one is already the block hash map for this
        // insertion. Skip this to avoid grazing.
        if ((clearing_ray ||
             global_voxel_idx != global_voxel_idx_to_point_indices.first) &&
            voxel_map.find(global_voxel_idx) != voxel_map.end()) {
          continue;
        }
      }

      // This is supposedly thread-safe...
      vxb::TsdfVoxel* tsdf_voxel = allocateStorageAndGetVoxelPtr(
          global_voxel_idx, &tsdf_block, &tsdf_block_idx);
      CHECK(tsdf_block);
      CHECK_NOTNULL(tsdf_voxel);

      updateTsdfVoxel(origin,
                      merged_point_G,
                      global_voxel_idx,
                      merged_color,
                      merged_weight,
                      tsdf_voxel);

      // TODO(Toni): if we add the "empty" label, this should not be necessary
      // but we will be doing quite a bit more of computation...
      // If voxel carving is enabled, then only allocate the label voxels
      // within semantic_truncation_distance_factor times the truncation
      // distance from the surface.
      if (config_.voxel_carving_enabled) {
        if (std::abs(tsdf_voxel->distance) <
            semantic_config_.semantic_truncation_distance_factor_ *
                config_.default_truncation_distance) {
          // if (ray_updated_voxel_empty) {
          //  //LOG(WARNING) << "This ray has already started updating empty"
          //  //                       "... How is this possible? TSDF is not
          //  what"
          //  //                       " you want dist_z is!";
          //  continue;
          //}
          SemanticVoxel* semantic_voxel = allocateStorageAndGetSemanticVoxelPtr(
              global_voxel_idx, &semantic_block, &semantic_block_idx);
          CHECK(semantic_block);
          CHECK_NOTNULL(semantic_voxel);
          updateSemanticVoxel(
              global_voxel_idx, semantic_label_frequencies, semantic_voxel);
        } else {
          VLOG(10) << "Not allocating voxel: too far from nearest object.";
        }
      } else {
        LOG_FIRST_N(WARNING, 1) << "Semantic voxel carving disabled.";
        // Allocate voxel no matter its distance to a surface. When we are
        // far away from a surface, we update the "empty" label.
        SemanticVoxel* semantic_voxel = allocateStorageAndGetSemanticVoxelPtr(
            global_voxel_idx, &semantic_block, &semantic_block_idx);
        CHECK(semantic_block);
        CHECK_NOTNULL(semantic_voxel);
        if (std::abs(tsdf_voxel->distance) <
            semantic_config_.semantic_truncation_distance_factor_ *
                config_.default_truncation_distance) {
          // if (ray_updated_voxel_empty) {
          //  //LOG(WARNING) << "This ray has already started updating empty"
          //  //                       "... How is this possible? TSDF is not
          //  what"
          //  //                       " you want dist_z is!";
          //  continue;
          //}
          updateSemanticVoxel(
              global_voxel_idx, semantic_label_frequencies, semantic_voxel);
        } else {
          // Update empty.
          ray_updated_voxel_empty = true;
          // If we are far from the surface start to update the semantic
          // labels of the voxels as "empty".
          SemanticProbabilities update_empty = SemanticProbabilities::Zero();
          // Use as evidence of this voxel being empty all the points used
          // inside this merged ray.
          CHECK_EQ(update_empty.rows(), semantic_label_frequencies.rows());
          // Maybe we should use
          // global_voxel_idx_to_point_indices.second.size()?
          // TODO(Toni): the sum of semantic_label_frequencies is -inf
          // sometimes.
          update_empty[0u] = 1.0;  // * semantic_label_frequencies.sum();
          CHECK_GE(update_empty.sum(), 1.0);
          updateSemanticVoxel(global_voxel_idx, update_empty, semantic_voxel);
        }
      }
    }
  }

  // TODO(Toni): Complete this function!!
  SemanticProbability computeMeasurementProbability(
      vxb::FloatingPoint ray_distance) {
    return 1.0;
  }

  // SHOULD BE THREAD SAFE. Updates semantic_voxel probabilities given
  // semantic_label measurement and confidence.
  void updateSemanticVoxel(const vxb::GlobalIndex& global_voxel_idx,
                           const SemanticProbabilities& measurement_frequencies,
                           SemanticVoxel* semantic_voxel) {
    CHECK_NOTNULL(semantic_voxel);
    // Lookup the mutex that is responsible for this voxel and lock it
    std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

    // Thread-safe region, you can modify the Voxel!
    CHECK_NOTNULL(semantic_voxel);

    // Calculate new probabilities given the measurement frequencies.
    updateSemanticVoxelProbabilities(measurement_frequencies,
                                     &semantic_voxel->semantic_priors);

    // Get MLE semantic label.
    calculateMaximumLikelihoodLabel(semantic_voxel->semantic_priors,
                                    &semantic_voxel->semantic_label);

    // Colorize according to current MLE semantic label.
    updateSemanticVoxelColor(semantic_voxel->semantic_label,
                             &semantic_voxel->color);
  }

 private:
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
      vxb::BlockIndex* last_block_idx) {
    CHECK_NOTNULL(last_semantic_block);
    CHECK_NOTNULL(last_block_idx);

    const vxb::BlockIndex block_idx = vxb::getBlockIndexFromGlobalVoxelIndex(
        global_voxel_idx, voxels_per_side_inv_);

    // TODO(margaritaG): citing Marius: this logic makes sure that if you
    // already have the right block pointer you don't need to go looking for it
    // again, so in order to make this logic effective you need to move the
    // block ptr and block idx outside of the while loop in the function calling
    // this.
    if ((block_idx != *last_block_idx) || (*last_semantic_block == nullptr)) {
      *last_semantic_block = semantic_layer_->getBlockPtrByIndex(block_idx);
      *last_block_idx = block_idx;
    }

    // If no block at this location currently exists, we allocate a temporary
    // voxel that will be merged into the map later
    if (*last_semantic_block == nullptr) {
      // To allow temp_label_block_map_ to grow we can only let
      // one thread in at once
      std::lock_guard<std::mutex> lock(temp_semantic_block_mutex_);

      typename vxb::Layer<SemanticVoxel>::BlockHashMap::iterator it =
          temp_semantic_block_map_.find(block_idx);
      if (it != temp_semantic_block_map_.end()) {
        *last_semantic_block = it->second;
      } else {
        auto insert_status = temp_semantic_block_map_.emplace(
            block_idx,
            std::make_shared<vxb::Block<SemanticVoxel>>(
                voxels_per_side_,
                voxel_size_,
                vxb::getOriginPointFromGridIndex(block_idx, block_size_)));

        CHECK(insert_status.second)
            << "Block already exists when allocating at "
            << block_idx.transpose();

        *last_semantic_block = insert_status.first->second;
      }
    }

    // Only used if someone calls the getAllUpdatedBlocks I believe.
    // (*last_semantic_block)->updated().set(); // do this when updating to
    // master.
    (*last_semantic_block)->updated() = true;

    const vxb::VoxelIndex local_voxel_idx =
        vxb::getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

    return &((*last_semantic_block)->getVoxelByVoxelIndex(local_voxel_idx));
  }

  // NOT THREAD SAFE
  void updateSemanticLayerWithStoredBlocks() {
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
  void updateSemanticVoxelProbabilities(
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
  void normalizeProbabilities(SemanticProbabilities* unnormalized_probs) const {
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
  inline void calculateMaximumLikelihoodLabel(
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
  void updateSemanticVoxelColor(const SemanticLabel& semantic_label,
                                HashableColor* semantic_voxel_color) const {
    CHECK_NOTNULL(semantic_voxel_color);
    // Do Not modify semantic_label_color_map_ here bcs we need to remain
    // thread-safe, and adding mutexes for coloring seems silly.
    // Precompute, for all possible SemanticLabels (255) a color.
    *semantic_voxel_color =
        semantic_config_.semantic_label_color_map_.at(semantic_label);
  }

 private:
  const SemanticConfig semantic_config_;

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

  // Layer with semantic information.
  vxb::Layer<SemanticVoxel>* semantic_layer_;
};

}  // Namespace kimera
