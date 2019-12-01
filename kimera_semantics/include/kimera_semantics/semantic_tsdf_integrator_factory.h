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

/**
 * @file   semantic_tsdf_integrator.h
 * @brief  Integrator of semantic and geometric information
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <memory>

#include <voxblox/integrator/tsdf_integrator.h>

#include "kimera_semantics/semantic_integrator_base.h"

namespace kimera {

//! Implemented types of semantic integrators.
enum class SemanticTsdfIntegratorType : int {
  kMerged = 0,
  kFast = 1,
};
const std::array<std::string, 2>
kSemanticTsdfIntegratorTypeNames = {{/*kMerged*/ "merged", /*kFast*/ "fast"}};

/// Creates a Semantic/TSDF integrator of the desired type.
class SemanticTsdfIntegratorFactory {
public:
  /**
   * @brief create an instance of a SemanticTsdfIntegrator which is casted to
   * the base class TsdfIntegratorBase for usage in a TsdfServer as if it was a
   * simple TsdfIntegrator
   * @param integrator_type_name type of integrator as specified in
   * kSemanticTsdfIntegratorTypeNames ("merged", "fast", ...)
   * @param config configuration for the TSDF integrator
   * @param semantic_config configuration for the Semantic integrator
   * @param tsdf_layer layer for the TSDF integrator
   * @param semantic_layer layer for the Semantic integrator
   * @return
   */
  static std::unique_ptr<vxb::TsdfIntegratorBase> create(
      const std::string& integrator_type_name,
      const vxb::TsdfIntegratorBase::Config& config,
      const SemanticIntegratorBase::SemanticConfig& semantic_config,
      vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
      vxb::Layer<SemanticVoxel>* semantic_layer);

  /**
   * @brief create same as `create` above but using enum for integrator_type.
   * @param integrator_type type of integrator as specified in the enum
   * SemanticTsdfIntegratorType ("merged", "fast", ...)
   * @param config configuration for the TSDF integrator
   * @param semantic_config configuration for the Semantic integrator
   * @param tsdf_layer layer for the TSDF integrator
   * @param semantic_layer layer for the Semantic integrator
   * @return
   */
  static std::unique_ptr<vxb::TsdfIntegratorBase> create(
      const SemanticTsdfIntegratorType& integrator_type,
      const vxb::TsdfIntegratorBase::Config& config,
      const SemanticIntegratorBase::SemanticConfig& semantic_config,
      vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
      vxb::Layer<SemanticVoxel>* semantic_layer);

private:
  /// Factories should be singletons, or rather, non instantiable objects.
  SemanticTsdfIntegratorFactory() = default;
  virtual ~SemanticTsdfIntegratorFactory() = default;

};

}  // Namespace kimera
