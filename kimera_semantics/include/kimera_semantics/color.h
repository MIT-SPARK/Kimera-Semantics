/**
 * @file   color.h
 * @brief  Hashable color information for displaying semantic labels
 * @author Antoni Rosinol
 */

#pragma once

#include <limits>
#include <string>
#include <unordered_map>

#include <voxblox/core/color.h>

#include "kimera_semantics/common.h"

namespace kimera {

struct HashableColor : public vxb::Color {
  HashableColor();
  HashableColor(const vxb::Color& color);
  HashableColor(uint8_t r, uint8_t g, uint8_t b);
  HashableColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

  bool operator==(const HashableColor& other) const;
  bool equal(const HashableColor& color) const;
};
typedef vxb::AlignedVector<HashableColor> HashableColors;

// For unordered map using Color as a Key.
struct ColorHasher {
  size_t operator()(const HashableColor& k) const;
};
typedef std::unordered_map<HashableColor, SemanticLabel, ColorHasher>
    ColorToSemanticLabelMap;
typedef std::unordered_map<SemanticLabel, HashableColor>
    SemanticLabelToColorMap;


// TODO(Toni): this is just to hack our way through the fact that our images
// are not label ids but just colors :( This is not used if the pointclouds
// you integrate have associated label ids. It is just for the case where
// you use its colors as ids.
class SemanticLabel2Color {
 public:
  SemanticLabel2Color(const std::string& filename);

  SemanticLabel getSemanticLabelFromColor(const HashableColor& color) const;

  HashableColor getColorFromSemanticLabel(
      const SemanticLabel& semantic_label) const;

  // Make these public if someone wants to access them directly.
  ColorToSemanticLabelMap color_to_semantic_label_;
  SemanticLabelToColorMap semantic_label_to_color_map_;
};

// Color map from semantic labels to colors.
// Precompute distinct but all possible colors.
// This is to avoid thread-saving updateSemanticColor...
inline SemanticLabelToColorMap getRandomSemanticLabelToColorMap() {
  SemanticLabelToColorMap semantic_label_color_map_;
  SemanticLabel max_possible_label = std::numeric_limits<SemanticLabel>::max();
  for (SemanticLabel i = 0; i < max_possible_label; i++) {
    semantic_label_color_map_[i] = vxb::randomColor();
  }
  // Make first colours easily distinguishable.
  CHECK_GE(semantic_label_color_map_.size(), 8u);
  CHECK_GE(semantic_label_color_map_.size(), kTotalNumberOfLabels);
  // TODO(Toni): Check it Matches with default value for SemanticVoxel!
  semantic_label_color_map_.at(0) = HashableColor::Gray();    // Label unknown
  semantic_label_color_map_.at(1) = HashableColor::Green();   // Label Ceiling
  semantic_label_color_map_.at(2) = HashableColor::Blue();    // Label Chair
  semantic_label_color_map_.at(3) = HashableColor::Purple();  // Label Floor
  semantic_label_color_map_.at(4) =
      HashableColor::Pink();  // Label Objects/Furniture/Chair
  semantic_label_color_map_.at(5) = HashableColor::Teal();    // Label Sofa
  semantic_label_color_map_.at(6) = HashableColor::Orange();  // Label Table
  semantic_label_color_map_.at(7) =
      HashableColor::Yellow();  // Label Wall/Window/TV/board/Picture
  return semantic_label_color_map_;
}

}  // namespace kimera
