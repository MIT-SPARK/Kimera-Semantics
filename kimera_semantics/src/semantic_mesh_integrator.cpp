/**
 * @file   semantic_mesh_integrator.cpp
 * @brief  Builds semantic 3D mesh
 * @author Antoni Rosinol
 */

#include "kimera_semantics/semantic_mesh_integrator.h"

#include <limits>
#include <list>
#include <memory>

#include <voxblox/core/block.h>
#include <voxblox/core/color.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

#include "kimera_semantics/color.h"
#include "kimera_semantics/common.h"
#include "kimera_semantics/semantic_voxel.h"

namespace kimera {

SemanticMeshIntegrator::SemanticMeshIntegrator(
    const vxb::MeshIntegratorConfig& config,
    const SemanticMeshConfig& semantic_config,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
    vxb::Layer<SemanticVoxel>* semantic_layer,
    vxb::MeshLayer* mesh_layer)
    : vxb::MeshIntegrator<vxb::TsdfVoxel>(config, tsdf_layer, mesh_layer),
      semantic_layer_mutable_ptr_(CHECK_NOTNULL(semantic_layer)),
      semantic_layer_const_ptr_(CHECK_NOTNULL(semantic_layer)),
      semantic_mesh_config_(semantic_config) {}

SemanticMeshIntegrator::SemanticMeshIntegrator(
    const vxb::MeshIntegratorConfig& config,
    const SemanticMeshConfig& semantic_config,
    const vxb::Layer<vxb::TsdfVoxel>& tsdf_layer,
    const vxb::Layer<SemanticVoxel>& semantic_layer,
    vxb::MeshLayer* mesh_layer)
    : vxb::MeshIntegrator<vxb::TsdfVoxel>(config, tsdf_layer, mesh_layer),
      semantic_layer_mutable_ptr_(nullptr),
      semantic_layer_const_ptr_(&semantic_layer),  // TODO(Toni) very dangerous
      semantic_mesh_config_(semantic_config) {}

/// Generates mesh from the tsdf and semantic layer.
bool SemanticMeshIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                          bool clear_updated_flag) {
  CHECK(!clear_updated_flag || ((sdf_layer_mutable_ != nullptr) &&
                                (semantic_layer_mutable_ptr_ != nullptr)))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf and "
         "label layers!";

  vxb::BlockIndexList all_tsdf_blocks;
  if (only_mesh_updated_blocks) {
    vxb::BlockIndexList all_label_blocks;
    sdf_layer_const_->getAllUpdatedBlocks(vxb::Update::kMesh, &all_tsdf_blocks);
    // TODO(Toni): not sure if we have to make our own update bit.
    semantic_layer_mutable_ptr_->getAllUpdatedBlocks(vxb::Update::kMesh,
                                                     &all_label_blocks);
    if (all_tsdf_blocks.size() == 0u && all_label_blocks.size() == 0u) {
      return false;
    }
    all_tsdf_blocks.insert(all_tsdf_blocks.end(),
                           all_label_blocks.begin(),
                           all_label_blocks.end());
  } else {
    LOG(WARNING) << "Mesh update all blocks not implemented for semantics...";
    sdf_layer_const_->getAllAllocatedBlocks(&all_tsdf_blocks);
  }

  // Allocate all the mesh memory
  for (const vxb::BlockIndex& block_index : all_tsdf_blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  std::unique_ptr<vxb::ThreadSafeIndex> index_getter(
      new vxb::MixedThreadSafeIndex(all_tsdf_blocks.size()));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &SemanticMeshIntegrator::generateMeshBlocksFunction,
        this,
        all_tsdf_blocks,
        clear_updated_flag,
        index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  return true;
}

void SemanticMeshIntegrator::generateMeshBlocksFunction(
    const vxb::BlockIndexList& all_tsdf_blocks,
    bool clear_updated_flag,
    vxb::ThreadSafeIndex* index_getter) {
  CHECK_NOTNULL(index_getter);
  CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr) ||
        (semantic_layer_mutable_ptr_ != nullptr))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf and "
         "label layers!";

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const vxb::BlockIndex& block_idx = all_tsdf_blocks.at(list_idx);
    updateMeshForBlock(block_idx);
    if (clear_updated_flag) {
      typename vxb::Block<vxb::TsdfVoxel>::Ptr tsdf_block =
          sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      typename vxb::Block<SemanticVoxel>::Ptr semantic_block =
          semantic_layer_mutable_ptr_->getBlockPtrByIndex(block_idx);
      // Use set in master
      DCHECK(tsdf_block);
      tsdf_block->updated() = false;
      DCHECK(semantic_block);
      semantic_block->updated() = false;
    }
  }
}

void SemanticMeshIntegrator::updateMeshForBlock(
    const vxb::BlockIndex& block_index) {
  vxb::Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh->clear();
  // This block should already exist, otherwise it makes no sense to update
  // the mesh for it. ;)
  vxb::Block<vxb::TsdfVoxel>::ConstPtr tsdf_block =
      sdf_layer_const_->getBlockPtrByIndex(block_index);
  vxb::Block<SemanticVoxel>::ConstPtr semantic_block =
      semantic_layer_const_ptr_->getBlockPtrByIndex(block_index);

  if (!tsdf_block) {
    LOG(FATAL) << "Trying to mesh a non-existent block at index: "
               << block_index.transpose();
    return;
  }

  extractBlockMesh(tsdf_block, mesh);
  // Update colors if needed.
  if (config_.use_color) {
    // CHECK(semantic_block) << "Non-existent semantic block!";
    updateMeshBlockColor(tsdf_block, semantic_block, mesh.get());
  }

  mesh->updated = true;
}

void SemanticMeshIntegrator::updateMeshBlockColor(
    vxb::Block<vxb::TsdfVoxel>::ConstPtr tsdf_block,
    vxb::Block<SemanticVoxel>::ConstPtr semantic_block,
    vxb::Mesh* mesh_block) {
  CHECK_NOTNULL(mesh_block);
  switch (semantic_mesh_config_.color_mode) {
    case ColorMode::kColor:
    case ColorMode::kNormals:
      CHECK(tsdf_block) << "Non-existent tsdf block!";
      MeshIntegrator::updateMeshColor(*tsdf_block, mesh_block);
      break;
    case ColorMode::kSemanticLabel:
    case ColorMode::kSemanticProbability:
      // CHECK(semantic_block) << "Non-existent semantic block!";
      if (semantic_block) {
        updateMeshColor(*semantic_block, mesh_block);
      } else {
        // TODO(Toni): there should be no missing semantic block:
        // for each tsdf there should be a semantic block, but for each
        // semantic block there should not be necessarily a tsdf block.
        VLOG(1) << "Missing semantic block for given tsdf block...";
        MeshIntegrator::updateMeshColor(*tsdf_block, mesh_block);
      }
      break;
    default:
      LOG(FATAL) << "Color scheme not recognized.";
  }
}

void SemanticMeshIntegrator::updateMeshColor(
    const vxb::Block<SemanticVoxel>& label_block,
    vxb::Mesh* mesh) {
  CHECK_NOTNULL(mesh);

  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search.
  for (size_t i = 0u; i < mesh->vertices.size(); ++i) {
    const vxb::Point& vertex = mesh->vertices[i];
    vxb::VoxelIndex voxel_index =
        label_block.computeVoxelIndexFromCoordinates(vertex);
    SemanticVoxel voxel;
    if (label_block.isValidVoxelIndex(voxel_index)) {
      voxel = label_block.getVoxelByVoxelIndex(voxel_index);
    } else {
      const typename vxb::Block<SemanticVoxel>::ConstPtr neighbor_block =
          semantic_layer_const_ptr_->getBlockPtrByCoordinates(vertex);
      voxel = neighbor_block->getVoxelByCoordinates(vertex);
    }
    getColorUsingColorScheme(
        semantic_mesh_config_.color_mode, voxel, &(mesh->colors[i]));
  }
}

void SemanticMeshIntegrator::getColorUsingColorScheme(
    const ColorMode& color_scheme,
    const SemanticVoxel& semantic_voxel,
    vxb::Color* color) {
  CHECK_NOTNULL(color);
  switch (color_scheme) {
    case ColorMode::kSemanticProbability:
      // TODO(Toni):
      // Might be a bit expensive to calc all these exponentials...
      *color = vxb::rainbowColorMap(std::exp(
          semantic_voxel.semantic_priors[semantic_voxel.semantic_label]));
      break;
    case ColorMode::kSemanticLabel:
      *color = semantic_mesh_config_.semantic_label_color_map.at(
          semantic_voxel.semantic_label);
      break;
    default:
      LOG(FATAL) << "Unknown mesh color scheme: "
                 << static_cast<std::underlying_type<ColorMode>::type>(
                        semantic_mesh_config_.color_mode);
  }
}

}  // namespace kimera
