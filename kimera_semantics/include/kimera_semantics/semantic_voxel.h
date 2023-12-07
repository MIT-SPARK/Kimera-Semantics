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
  //! Current MLE semantic label
  SemanticLabel semantic_label = 0u;
  //! Log-likelihood priors of each label
  SemanticProbabilities semantic_priors;
  //! Current color of label
  HashableColor color = HashableColor::Gray();
  //! Whether or not the voxel has been initialized
  bool empty = true;
};

}  // namespace kimera
