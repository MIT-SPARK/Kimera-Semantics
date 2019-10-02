#pragma once

#include <string>
#include <iostream>
#include <fstream>

#include <voxblox/core/color.h>

#include "kimera_semantics/common.h"
#include "kimera_semantics/csv_iterator.h"
#include "kimera_semantics/color.h"

namespace kimera {

struct HashableColor : public vxb::Color {
  HashableColor(const Color& color) : Color(color) {};
  HashableColor() : Color() {};
  HashableColor(uint8_t r, uint8_t g, uint8_t b)
      : HashableColor(r, g, b, 255) {};
  HashableColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
      : Color(r, g, b, a) {};

  bool operator==(const HashableColor& other) const {
    return (r == other.r && g == other.g &&
            b == other.b && a == other.a);
  }

  bool equal(const HashableColor& color) const {
    return r == color.r && g == color.g && b == color.b && a == color.a;
  }
};
typedef vxb::AlignedVector<HashableColor> HashableColors;

// For unordered map using Color as a Key.
struct ColorHasher {
  size_t operator()(const HashableColor& k) const {
    // Compute individual hash values for first,
    // second and third and combine them using XOR
    // and bit shifting:
    // TODO(Toni): use alpha value as well!!
    return ((std::hash<uint8_t>()(k.r)
             ^ (std::hash<uint8_t>()(k.g) << 1)) >> 1)
             ^ (std::hash<uint8_t>()(k.b) << 1);
  }
};
typedef std::unordered_map<HashableColor, SemanticLabel, ColorHasher>
    ColorToSemanticLabelMap;
typedef std::unordered_map<SemanticLabel, HashableColor>
    SemanticLabelToColorMap;

class SemanticLabel2Color {
public:
  SemanticLabel2Color(const std::string& filename)
    : color_to_semantic_label_(),
      semantic_label_to_color_() {
    std::ifstream file(filename.c_str());
    CHECK(file.good()) << "Couldn't open file: " << filename.c_str();
    size_t row_number = 1;
    for(CSVIterator loop(file); loop != CSVIterator(); ++loop) {
        // We expect the CSV to have header:
        // 0   , 1  , 2    , 3   , 4    , 5
        // name, red, green, blue, alpha, id
        CHECK_EQ(loop->size(), 6) << "Row " << row_number << " is invalid.";
        uint8_t r = std::atoi((*loop)[1].c_str());
        uint8_t g = std::atoi((*loop)[2].c_str());
        uint8_t b = std::atoi((*loop)[3].c_str());
        uint8_t a = std::atoi((*loop)[4].c_str());
        uint8_t id = std::atoi((*loop)[5].c_str());
        HashableColor rgba = HashableColor(r, g, b, a);
        semantic_label_to_color_[id] = rgba;
        color_to_semantic_label_[rgba] = id;
        row_number++;
    }
    // TODO(Toni): remove
    // Assign color 255,255,255 to unknown object 0u
    color_to_semantic_label_[HashableColor::White()] = 0u;
  }

  SemanticLabel getSemanticLabelFromColor(const HashableColor& color) const {
    return color_to_semantic_label_.at(color);
  }

  HashableColor getColorFromSemanticLabel(const SemanticLabel& semantic_label) const {
    return semantic_label_to_color_.at(semantic_label);
  }

  // Make these public if someone wants to access them directly.
  ColorToSemanticLabelMap color_to_semantic_label_;
  SemanticLabelToColorMap semantic_label_to_color_;
};

// Color map from semantic labels to colors.
// Precompute distinct but all possible colors.
// This is to avoid thread-saving updateSemanticColor...
inline SemanticLabelToColorMap getRandomSemanticLabelToColorMap() {
  SemanticLabelToColorMap semantic_label_color_map_;
  SemanticLabel max_possible_label =
      std::numeric_limits<SemanticLabel>::max();
  for (SemanticLabel i = 0; i < max_possible_label; i++) {
    semantic_label_color_map_[i] = vxb::randomColor();
  }
  // Make first colours easily distinguishable.
  CHECK_GE(semantic_label_color_map_.size(), 8);
  CHECK_GE(semantic_label_color_map_.size(), kTotalNumberOfLabels);
  // TODO(Toni): Check it Matches with default value for SemanticVoxel!
  semantic_label_color_map_.at(0) = HashableColor::Gray();   // Label unknown
  semantic_label_color_map_.at(1) = HashableColor::Green();  // Label Ceiling
  semantic_label_color_map_.at(2) = HashableColor::Blue();   // Label Chair
  semantic_label_color_map_.at(3) = HashableColor::Purple(); // Label Floor
  semantic_label_color_map_.at(4) = HashableColor::Pink(); // Label Objects/Furniture/Chair
  semantic_label_color_map_.at(5) = HashableColor::Teal(); // Label Sofa
  semantic_label_color_map_.at(6) = HashableColor::Orange(); // Label Table
  semantic_label_color_map_.at(7) = HashableColor::Yellow(); // Label Wall/Window/TV/board/Picture
  return semantic_label_color_map_;
}

}
