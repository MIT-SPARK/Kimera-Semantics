/**
 * @file   common.h
 * @brief  Common definitions
 * @author Antoni Rosinol
 */

#pragma once

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox/utils/approx_hash_array.h>

namespace kimera {

namespace vxb = voxblox;

// Consider id 0 to be the `unknown' label, for which we don't update the
// log-likelihood for that measurement.
static constexpr uint8_t kUnknownSemanticLabelId = 0u;

typedef uint8_t SemanticLabel;
typedef vxb::AlignedVector<SemanticLabel> SemanticLabels;

typedef vxb::FloatingPoint SemanticProbability;
typedef Eigen::Matrix<SemanticProbability, Eigen::Dynamic, 1>
    SemanticProbabilities;

// A `#Labels X #Labels` Eigen matrix where each `j` column represents the
// probability of observing label `j` when current label is `i`, where `i`
// is the row index of the matrix.
typedef Eigen::Matrix<SemanticProbability, Eigen::Dynamic, Eigen::Dynamic>
    SemanticLikelihoodFunction;

typedef vxb::LongIndexHashMapType<vxb::AlignedVector<size_t>>::type VoxelMap;
typedef VoxelMap::value_type VoxelMapElement;

}  // namespace kimera
