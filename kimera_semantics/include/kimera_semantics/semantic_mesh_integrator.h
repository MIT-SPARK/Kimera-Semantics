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
  enum class ColorMode {
    kColor = 0,
    kNormals,
    kSemanticLabel,
    kSemanticProbability,
  };

  struct SemanticMeshConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // In Log odds!
    // -std::numeric_limits<SemanticProbability>::max(); represents a prob of 0.
    SemanticProbability min_probability =
        -std::numeric_limits<SemanticProbability>::max();

    // Color mode to use for the mesh color.
    // By default the mesh encodes the labels.
    ColorMode color_mode = ColorMode::kSemanticLabel;

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
  bool generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag);

 protected:
  // Needs to be thread-safe
  void generateMeshBlocksFunction(const vxb::BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  vxb::ThreadSafeIndex* index_getter);

  virtual void updateMeshForBlock(const vxb::BlockIndex& block_index) override;

  void updateMeshBlockColor(vxb::Block<vxb::TsdfVoxel>::ConstPtr tsdf_block,
                            vxb::Block<SemanticVoxel>::ConstPtr semantic_block,
                            vxb::Mesh* mesh_block);

  void updateMeshColor(const vxb::Block<SemanticVoxel>& label_block,
                       vxb::Mesh* mesh);

  void getColorUsingColorScheme(const ColorMode& color_scheme,
                                const SemanticVoxel& semantic_voxel,
                                vxb::Color* color);

 protected:
  // Having both a const and a mutable pointer to the layer allows this
  // integrator to work both with a const layer (in case you don't want to
  // clear the updated flag) and mutable layer (in case you do want to clear
  // the updated flag).
  vxb::Layer<SemanticVoxel>* semantic_layer_mutable_ptr_;
  const vxb::Layer<SemanticVoxel>* semantic_layer_const_ptr_;
  const SemanticMeshConfig semantic_mesh_config_;
};

}  // namespace kimera
