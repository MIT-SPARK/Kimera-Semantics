#pragma once

#include <voxblox/core/common.h>

namespace kimera {

namespace vxb = voxblox;

typedef uint8_t SemanticLabel;
typedef vxb::AlignedVector<SemanticLabel> SemanticLabels;
// The size of this array determines how many semantic labels SemanticVoxblox
// supports.
// TODO(Toni): parametrize this, although that means it becomes unknown at
// compile time...
static constexpr size_t kTotalNumberOfLabels = 20;
typedef vxb::FloatingPoint SemanticProbability;
typedef Eigen::Matrix<SemanticProbability, kTotalNumberOfLabels, 1>
    SemanticProbabilities;
// A `#Labels X #Labels` Eigen matrix where each `j` column represents the
// probability of observing label `j` when current label is `i`, where `i`
// is the row index of the matrix.
typedef Eigen::
    Matrix<SemanticProbability, kTotalNumberOfLabels, kTotalNumberOfLabels>
        SemanticLikelihoodFunction;
typedef std::unordered_map<SemanticLabel, vxb::Color> SemanticLabelToColorMap;

}  // namespace kimera
