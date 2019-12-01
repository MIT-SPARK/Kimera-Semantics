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

/**
 * @file   semantic_tsdf_integrator_factory.cpp
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#include "kimera_semantics/semantic_tsdf_integrator_factory.h"

#include "kimera_semantics/semantic_tsdf_integrator_fast.h"
#include "kimera_semantics/semantic_tsdf_integrator_merged.h"

namespace kimera {

std::unique_ptr<vxb::TsdfIntegratorBase>
SemanticTsdfIntegratorFactory::create(
    const std::string& integrator_type_name,
    const vxb::TsdfIntegratorBase::Config& config,
    const SemanticIntegratorBase::SemanticConfig& semantic_config,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
    vxb::Layer<SemanticVoxel>* semantic_layer) {
  CHECK(!integrator_type_name.empty());

  int integrator_type = 0;
  for (const std::string& valid_integrator_type_name :
       kSemanticTsdfIntegratorTypeNames) {
    if (integrator_type_name == valid_integrator_type_name) {
      return create(static_cast<SemanticTsdfIntegratorType>(integrator_type),
                    config, semantic_config, tsdf_layer, semantic_layer);
    }
    ++integrator_type;
  }
  LOG(FATAL) << "Unknown TSDF integrator type: " << integrator_type_name;
  return nullptr;
}

std::unique_ptr<vxb::TsdfIntegratorBase>
SemanticTsdfIntegratorFactory::create(
    const SemanticTsdfIntegratorType& integrator_type,
    const vxb::TsdfIntegratorBase::Config& config,
    const SemanticIntegratorBase::SemanticConfig& semantic_config,
    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
    vxb::Layer<SemanticVoxel>* semantic_layer) {
  CHECK_NOTNULL(tsdf_layer);
  switch (integrator_type) {
  case SemanticTsdfIntegratorType::kFast:
    return kimera::make_unique<FastSemanticTsdfIntegrator>(
          config, semantic_config, tsdf_layer, semantic_layer);
    break;
  case SemanticTsdfIntegratorType::kMerged:
    return kimera::make_unique<MergedSemanticTsdfIntegrator>(
          config, semantic_config, tsdf_layer, semantic_layer);
    break;
  default:
    LOG(FATAL) << "Unknown Semantic/TSDF integrator type: "
               << static_cast<int>(integrator_type);
    break;
  }
  return nullptr;
}

}  // Namespace kimera
