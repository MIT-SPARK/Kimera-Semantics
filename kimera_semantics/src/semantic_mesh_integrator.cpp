// NOTE: Most code is derived from voxblox: github.com/ethz-asl/voxblox
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Further, this part of the code is derived from OpenChisel
// https://github.com/personalrobotics/OpenChisel
// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
      semantic_layer_mutable_(CHECK_NOTNULL(semantic_layer)),
      semantic_layer_const_(CHECK_NOTNULL(semantic_layer)),
      semantic_mesh_config_(semantic_config) {}

SemanticMeshIntegrator::SemanticMeshIntegrator(
    const vxb::MeshIntegratorConfig& config,
    const SemanticMeshConfig& semantic_config,
    const vxb::Layer<vxb::TsdfVoxel>& tsdf_layer,
    const vxb::Layer<SemanticVoxel>& semantic_layer,
    vxb::MeshLayer* mesh_layer)
    : vxb::MeshIntegrator<vxb::TsdfVoxel>(config, tsdf_layer, mesh_layer),
      semantic_layer_mutable_(nullptr),
      semantic_layer_const_(&semantic_layer),  // TODO(Toni) very dangerous
      semantic_mesh_config_(semantic_config) {}

/// Generates mesh from the tsdf and semantic layer.
void SemanticMeshIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                          bool clear_updated_flag) {
  CHECK(!clear_updated_flag || ((sdf_layer_mutable_ != nullptr) &&
                                (semantic_layer_mutable_ != nullptr)))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf "
      << "layer!";

  vxb::BlockIndexList all_tsdf_blocks;
  vxb::BlockIndexList all_semantic_blocks;
  if (only_mesh_updated_blocks) {
    sdf_layer_const_->getAllUpdatedBlocks(vxb::Update::kMesh, &all_tsdf_blocks);
    semantic_layer_const_->getAllUpdatedBlocks(vxb::Update::kMesh, &all_semantic_blocks);
  } else {
    sdf_layer_const_->getAllAllocatedBlocks(&all_tsdf_blocks);
    semantic_layer_const_->getAllAllocatedBlocks(&all_semantic_blocks);
  }
  all_tsdf_blocks.insert(all_tsdf_blocks.end(), all_semantic_blocks.begin(), all_semantic_blocks.end());

  // Allocate all the mesh memory
  for (const vxb::BlockIndex& block_index : all_tsdf_blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  std::unique_ptr<vxb::ThreadSafeIndex> index_getter(
      new vxb::MixedThreadSafeIndex(all_tsdf_blocks.size()));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &SemanticMeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks,
        clear_updated_flag, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
}

void SemanticMeshIntegrator::generateMeshBlocksFunction(
    const vxb::BlockIndexList& all_tsdf_blocks,
    bool clear_updated_flag,
    vxb::ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);
  CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr) ||
        (semantic_layer_mutable_ != nullptr))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf "
         "layer!";

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const vxb::BlockIndex& block_idx = all_tsdf_blocks.at(list_idx);
    updateMeshForBlock(block_idx);
    if (clear_updated_flag) {
      typename vxb::Block<vxb::TsdfVoxel>::Ptr tsdf_block =
          sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      tsdf_block->updated().reset(vxb::Update::kMesh);
      typename vxb::Block<SemanticVoxel>::Ptr semantic_block =
          semantic_layer_mutable_->getBlockPtrByIndex(block_idx);
      semantic_block->updated().reset(vxb::Update::kMesh);
    }
  }
}

void SemanticMeshIntegrator::updateMeshForBlock(
    const vxb::BlockIndex& block_index) {
  vxb::Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh->clear();
  // This block should already exist, otherwise it makes no sense to update
  // the mesh for it. ;)
  typename vxb::Block<vxb::TsdfVoxel>::ConstPtr tsdf_block =
      sdf_layer_const_->getBlockPtrByIndex(block_index);
  typename vxb::Block<SemanticVoxel>::ConstPtr semantic_block =
      semantic_layer_const_->getBlockPtrByIndex(block_index);

  if (!tsdf_block) {
    LOG(ERROR) << "Trying to mesh a non-existent tsdf block at index: "
               << block_index.transpose();
    return;
  }
  extractBlockMesh(tsdf_block, mesh);
  // Update colors if needed.
  if (config_.use_color) {
    if (semantic_block) {
      updateMeshColor(*semantic_block, mesh.get());
    } else {
      MeshIntegrator::updateMeshColor(*tsdf_block, mesh.get());
    }
  }

  mesh->updated = true;
}

void SemanticMeshIntegrator::updateMeshColor(
    const vxb::Block<SemanticVoxel>& block,
    vxb::Mesh* mesh) {
  DCHECK(mesh != nullptr);

  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search.
  for (size_t i = 0; i < mesh->vertices.size(); ++i) {
    const vxb::Point& vertex = mesh->vertices[i];
    vxb::VoxelIndex voxel_index = block.computeVoxelIndexFromCoordinates(vertex);
    if (block.isValidVoxelIndex(voxel_index)) {
      const SemanticVoxel& voxel = block.getVoxelByVoxelIndex(voxel_index);
      getColorUsingColorMode(
            semantic_mesh_config_.color_mode, voxel, &(mesh->colors[i]));
    } else {
      const typename vxb::Block<SemanticVoxel>::ConstPtr neighbor_block =
          semantic_layer_const_->getBlockPtrByCoordinates(vertex);
      const SemanticVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
      getColorUsingColorMode(
            semantic_mesh_config_.color_mode, voxel, &(mesh->colors[i]));
    }
  }
}

void SemanticMeshIntegrator::getColorUsingColorMode(
    const ColorMode& color_mode,
    const SemanticVoxel& semantic_voxel,
    vxb::Color* color) {
  CHECK_NOTNULL(color);
  switch (color_mode) {
    case ColorMode::kSemanticProbability:
      // TODO(Toni): Might be a bit expensive to calc all these exponentials...
      *color = vxb::rainbowColorMap(std::exp(
          semantic_voxel.semantic_priors[semantic_voxel.semantic_label]));
      break;
    case ColorMode::kSemantic:
      *color = semantic_mesh_config_.semantic_label_color_map.at(
          semantic_voxel.semantic_label);
      break;
    default:
      *color = semantic_mesh_config_.semantic_label_color_map.at(
            semantic_voxel.semantic_label);
      break;
  }
}

}  // namespace kimera
