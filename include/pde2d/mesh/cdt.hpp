#pragma once

#include "pde2d/mesh/components.hpp"

namespace pde2d::mesh {

// Constrained Delaunay triangulation of the polygon: the points are
// inserted one by one into a large enclosing triangle (keeping it
// Delaunay by flips), missing polygon edges are recovered by flips,
// triangles outside the domain are removed, and a final pass of flips
// makes every interior edge locally Delaunay. Adds no points. Metrics:
// "triangles", "insertion_flips", "recovery_flips", "final_flips".
class Cdt final : public InitialTriangulator {
public:
  std::string name() const override { return "cdt"; }
  TriangulatorCapabilities capabilities() const override {
    return {.supports_holes = true, .supports_several_pieces = true};
  }
  WorkingMesh run(const BoundaryPolygon &polygon,
                  StageReport &report) const override;
};

} // namespace pde2d::mesh
