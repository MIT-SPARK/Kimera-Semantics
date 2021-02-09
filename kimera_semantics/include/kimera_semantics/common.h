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

typedef uint8_t SemanticLabel;
typedef vxb::AlignedVector<SemanticLabel> SemanticLabels;
// The size of this array determines how many semantic labels SemanticVoxblox
// supports.

//Dynamic size Semantic Probabilities by David R.
typedef vxb::FloatingPoint SemanticProbability;
typedef Eigen::Matrix<SemanticProbability, Eigen::Dynamic, 1>
    SemanticProbabilities;
// A `#Labels X #Labels` Eigen matrix where each `j` column represents the
// probability of observing label `j` when current label is `i`, where `i`
// is the row index of the matrix.
typedef Eigen::
    Matrix<SemanticProbability, Eigen::Dynamic, Eigen::Dynamic>
        SemanticLikelihoodFunction;

typedef vxb::LongIndexHashMapType<vxb::AlignedVector<size_t>>::type VoxelMap;
typedef VoxelMap::value_type VoxelMapElement;

// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace kimera
