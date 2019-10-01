#pragma once

#include <kimera_semantics/common.h>

namespace kimera {

struct SemanticVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize voxel to unknown label.
  SemanticLabel semantic_label = 0u;
  // Initialize voxel to uniform probability.
  // Use log odds! So uniform ditribution of 1/kTotalNumberOfLabels,
  // should be std::log(1/kTotalNumberOfLabels)
  SemanticProbabilities semantic_priors =
      // SemanticProbabilities::Constant(std::log(1 / kTotalNumberOfLabels));
      SemanticProbabilities::Constant(-0.60205999132);
  // Initialize voxel with gray color
  // Make sure that all color maps agree on semantic label 0u -> gray
  vxb::Color color = vxb::Color::Gray();
};

}
