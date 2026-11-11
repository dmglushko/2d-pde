#include "pde2d/mesh/components.hpp"

#include <stdexcept>

namespace pde2d::mesh {

void check_compatibility(
    const Refiner *refiner,
    const std::vector<std::shared_ptr<const Improver>> &improvers) {
  // Whether the mesh is known to be Delaunay at this point of the
  // pipeline, and which component made it so.
  bool delaunay = false;
  std::string source;
  if (refiner) {
    auto caps = refiner->capabilities();
    if (const LocalImprover *local = refiner->local_improver()) {
      auto fx = local->effects();
      if (fx.flips_edges && !caps.tolerates_flips)
        throw std::invalid_argument(refiner->name() +
                                    " does not tolerate edge flips, but its "
                                    "local improver " +
                                    local->name() + " flips edges");
      if (fx.moves_nodes && !caps.tolerates_node_moves)
        throw std::invalid_argument(refiner->name() +
                                    " does not tolerate node moves, but its "
                                    "local improver " +
                                    local->name() + " moves nodes");
      if (fx.makes_delaunay && caps.maintains_delaunay)
        throw std::invalid_argument(refiner->name() +
                                    " already maintains Delaunay; its local "
                                    "improver " +
                                    local->name() + " is redundant");
    }
    if (caps.maintains_delaunay) {
      delaunay = true;
      source = refiner->name();
    }
  }
  for (const auto &improver : improvers) {
    if (!improver)
      throw std::invalid_argument("Improver list contains a null pointer");
    auto fx = improver->effects();
    if (fx.makes_delaunay && delaunay)
      throw std::invalid_argument(source + " already maintains Delaunay; " +
                                  improver->name() + " is redundant here");
    if (fx.makes_delaunay) {
      delaunay = true;
      source = improver->name();
    } else if (fx.flips_edges || fx.moves_nodes) {
      delaunay = false;
    }
  }
}

} // namespace pde2d::mesh
