/**
 * @file   semantic_voxel.h
 * @brief  Definition of what a semantic voxel is
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera_semantics/color.h"
#include "kimera_semantics/common.h"

namespace kimera {

    struct SemanticVoxel {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Initialize voxel to unknown label.
        SemanticLabel semantic_label = 0u;
        // Initialize voxel to uniform probability.
        // Use log odds! So uniform ditribution of 1/total_number_of_labels_,
        // should be std::log(1/total_number_of_labels_)
        size_t total_number_of_layers_;

        // SemanticProbabilities::Constant(std::log(1 / total_number_of_labels_));
        SemanticProbabilities semantic_priors;

        // Initialize voxel with gray color
        // Make sure that all color maps agree on semantic label 0u -> gray
        HashableColor color = HashableColor::Gray();
    };

}  // namespace kimera