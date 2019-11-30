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
 * @file   semantic_mesh_integrator.h
 * @brief  Builds semantic 3D mesh
 * @author Antoni Rosinol
 */

#pragma once

#include <limits>

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

class SemanticMeshIntegrator : public vxb::MeshIntegrator<vxb::TsdfVoxel> {
 public:

  // Similar to Voxblox:
  //  enum ColorMode {
  //   kColor = 0,
  //   kHeight,
  //   kNormals,
  //   kGray,
  //   kLambert,
  //   kLambertColor
  // };
  // But modernize using enum class.
  enum class ColorMode {
    kColor = 0,
    kNormals = 1,
    kSemantic = 2,
    kSemanticProbability = 3,
  };

  struct SemanticMeshConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // In Log odds!
    // -std::numeric_limits<SemanticProbability>::max(); represents a prob of 0.
    SemanticProbability min_probability =
        -std::numeric_limits<SemanticProbability>::max();

    /// How to color the mesh.
    ColorMode color_mode = ColorMode::kSemantic;

    // Map from semantic label to actual color.
    SemanticLabelToColorMap semantic_label_color_map =
        getRandomSemanticLabelToColorMap();
  };

  SemanticMeshIntegrator(const vxb::MeshIntegratorConfig& config,
                         const SemanticMeshConfig& semantic_config,
                         vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
                         vxb::Layer<SemanticVoxel>* semantic_layer,
                         vxb::MeshLayer* mesh_layer);

  SemanticMeshIntegrator(const vxb::MeshIntegratorConfig& config,
                         const SemanticMeshConfig& semantic_config,
                         const vxb::Layer<vxb::TsdfVoxel>& tsdf_layer,
                         const vxb::Layer<SemanticVoxel>& semantic_layer,
                         vxb::MeshLayer* mesh_layer);

  /// Generates mesh from the tsdf and semantic layer.
  void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag);

 protected:
  // Needs to be thread-safe
  void generateMeshBlocksFunction(const vxb::BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  vxb::ThreadSafeIndex* index_getter);

  virtual void updateMeshForBlock(const vxb::BlockIndex& block_index) override;

  void updateMeshColor(const vxb::Block<voxblox::TsdfVoxel>& tsdf_block,
                       const vxb::Block<SemanticVoxel>& semantic_block,
                       vxb::Mesh* mesh_block);

  void updateMeshColor(const vxb::Block<SemanticVoxel>& block,
                       vxb::Mesh* mesh);

  void getColorUsingColorMode(const ColorMode& color_mode,
                                const SemanticVoxel& semantic_voxel,
                                vxb::Color* color);

 protected:
  // Having both a const and a mutable pointer to the layer allows this
  // integrator to work both with a const layer (in case you don't want to
  // clear the updated flag) and mutable layer (in case you do want to clear
  // the updated flag).
  vxb::Layer<SemanticVoxel>* semantic_layer_mutable_;
  const vxb::Layer<SemanticVoxel>* semantic_layer_const_;
  const SemanticMeshConfig semantic_mesh_config_;
};

}  // namespace kimera
