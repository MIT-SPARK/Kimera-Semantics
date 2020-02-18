#include <voxblox/simulation/simulation_world.h>

#include <map>
#include <array>

#include "kimera_semantics/color.h"
#include "kimera_semantics/semantic_voxel.h"

namespace kimera {

// The size of this array determines how many semantic labels SemanticVoxblox
// supports.
typedef std::unordered_map<SemanticLabel, vxb::Color> SemanticColors;

class SemanticSimulationWorld : public vxb::SimulationWorld {
 public:
  SemanticSimulationWorld()
      : vxb::SimulationWorld() {
    // Prefill with the empty color.
    semantic_colors_[0] = vxb::Color::Gray();
  }
  virtual ~SemanticSimulationWorld() {}

  // Selects semantic labels and fills the semantic_colors_ structure
  void selectSemanticLabel(const vxb::Object::Type& object_type,
                           SemanticLabel* semantic_label);

  void generateSemanticSdfFromWorld(vxb::FloatingPoint max_dist,
                                    vxb::Layer<SemanticVoxel>* layer);

  void setSemanticVoxel(const SemanticLabel& label, SemanticVoxel* voxel) const;

  SemanticColors semantic_colors_;
};

}  // namespace kimera
