#include "kimera_semantics/simulation/semantic_simulation_world.h"

namespace kimera {

void SemanticSimulationWorld::selectSemanticLabel(
    const vxb::Object::Type& object_type,
    SemanticLabel* semantic_label) {
  CHECK_NOTNULL(semantic_label);
  switch (object_type) {
    case vxb::Object::Type::kSphere: {
      *semantic_label = 1u;
      semantic_colors_[*semantic_label] = vxb::Color::Red();
      break;
    }
    case vxb::Object::Type::kCube: {
      *semantic_label = 2u;
      semantic_colors_[*semantic_label] = vxb::Color::Blue();
      break;
    }
    case vxb::Object::Type::kPlane: {
      *semantic_label = 3u;
      semantic_colors_[*semantic_label] = vxb::Color::Green();
      break;
    }
    case vxb::Object::Type::kCylinder: {
      *semantic_label = 4u;
      semantic_colors_[*semantic_label] = vxb::Color::Pink();
      break;
    }
    default:
      LOG(FATAL) << "Object type not recognized.";
  }
}

void SemanticSimulationWorld::generateSemanticSdfFromWorld(
    vxb::FloatingPoint max_dist,
    vxb::Layer<SemanticVoxel>* layer) {
  vxb::timing::Timer sim_timer("sim/generate_semantic_sdf");

  CHECK_NOTNULL(layer);
  // Iterate over every voxel in the layer and compute its distance to all
  // objects.

  // Get all blocks within bounds. For now, only respect bounds approximately:
  // that is, up to block boundaries.
  vxb::FloatingPoint block_size = layer->block_size();
  vxb::FloatingPoint half_block_size = block_size / 2.0;

  vxb::BlockIndexList blocks;
  vxb::Point min_bound = getMinBound();
  vxb::Point max_bound = getMaxBound();
  for (vxb::FloatingPoint x = min_bound.x() - half_block_size;
       x <= max_bound.x() + half_block_size;
       x += block_size) {
    for (vxb::FloatingPoint y = min_bound.y() - half_block_size;
         y <= max_bound.y() + half_block_size;
         y += block_size) {
      for (vxb::FloatingPoint z = min_bound.z() - half_block_size;
           z <= max_bound.z() + half_block_size;
           z += block_size) {
        blocks.push_back(
            layer->computeBlockIndexFromCoordinates(vxb::Point(x, y, z)));
      }
    }
  }

  // Iterate over all blocks filling this stuff in.
  for (const vxb::BlockIndex& block_index : blocks) {
    vxb::Block<SemanticVoxel>::Ptr block =
        layer->allocateBlockPtrByIndex(block_index);
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      SemanticVoxel& voxel = block->getVoxelByLinearIndex(i);
      vxb::Point coords = block->computeCoordinatesFromLinearIndex(i);
      // Check that it's in bounds, otherwise skip it.
      if (!(coords.x() >= min_bound.x() && coords.x() <= max_bound.x() &&
            coords.y() >= min_bound.y() && coords.y() <= max_bound.y() &&
            coords.z() >= min_bound.z() && coords.z() <= max_bound.z())) {
        continue;
      }

      // Iterate over all objects and get distances to this thing.
      vxb::FloatingPoint voxel_dist = max_dist;
      SemanticLabel semantic_label = 0u;
      for (const std::unique_ptr<vxb::Object>& object : objects_) {
        vxb::FloatingPoint object_dist = object->getDistanceToPoint(coords);
        if (object_dist < voxel_dist) {
          voxel_dist = object_dist;
          selectSemanticLabel(object->getType(), &semantic_label);
        }
      }

      // Then update the thing.
      voxel_dist = std::max(voxel_dist, -max_dist);
      setSemanticVoxel(semantic_label, &voxel);
    }
  }
}

void SemanticSimulationWorld::setSemanticVoxel(const SemanticLabel& label,
                                               SemanticVoxel* voxel) const {
  CHECK_NOTNULL(voxel);
  voxel->semantic_label = label;
  CHECK_LT(label, voxel->semantic_priors.size());
  // Set as ground-truth: prob of current label = 1, -log(1)=0
  voxel->semantic_priors[label] = 0.0;
  const auto& it = semantic_colors_.find(label);
  CHECK(it != semantic_colors_.end());
  voxel->color = it->second;
}

}  // namespace kimera
