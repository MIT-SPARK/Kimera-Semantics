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

  size_t getNumLabels() const;
};

// Color map from semantic labels to colors.
// Precompute distinct but all possible colors.
// This is to avoid thread-saving updateSemanticColor...
SemanticLabelToColorMap getRandomSemanticLabelToColorMap(size_t num_labels);

}  // namespace kimera
