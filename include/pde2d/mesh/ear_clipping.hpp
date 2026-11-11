#pragma once

#include "pde2d/mesh/components.hpp"

namespace pde2d::mesh {

// Triangulates each piece of the domain (a loop of even depth with its
// holes) on its own: the holes are bridged into the outer loop, then ears
// are cut off one at a time, best-shaped ear first. Adds no points; the
// result is generally not Delaunay. Metrics: "triangles", "pieces",
// "bridges".
class EarClipping final : public InitialTriangulator {
public:
  std::string name() const override { return "ear_clipping"; }
  TriangulatorCapabilities capabilities() const override {
    return {.supports_holes = true, .supports_several_pieces = true};
  }
  WorkingMesh run(const BoundaryPolygon &polygon,
                  StageReport &report) const override;
};

} // namespace pde2d::mesh
