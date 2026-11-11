#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct RuppertOptions {
  // Smallest angle the refined triangles may have, in degrees, in (0, 34].
  // Termination is proven up to about 20.7 degrees for inputs without
  // angles below 60 degrees, and usual in practice up to about 33.
  double min_angle_degrees = 20.0;
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Ruppert's Delaunay refinement. First makes its input (constrained)
// Delaunay by flips, then repeats:
//   - split a boundary edge that is encroached (a node lies inside its
//     diametral circle) or longer than max_edge_length;
//   - for a triangle with an angle below the bound or an edge longer than
//     max_edge_length, insert its circumcenter, unless that point would
//     encroach boundary edges or lies beyond one, in which case those
//     boundary edges are split instead.
// Every insertion is followed by flips, so the mesh stays constrained
// Delaunay. A boundary edge with a curve link is split at the curve point.
// Metrics: "initial_flips", "flips", "boundary_splits", "circumcenters".
class Ruppert final : public Refiner {
public:
  explicit Ruppert(RuppertOptions options = {});
  const RuppertOptions &options() const { return options_; }

  std::string name() const override { return "ruppert"; }
  RefinerCapabilities capabilities() const override {
    // It keeps its own Delaunay invariant, so it accepts no outside
    // changes during refinement.
    return {.tolerates_flips = false,
            .tolerates_node_moves = false,
            .maintains_delaunay = true};
  }
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           StageReport &report) const override;

private:
  RuppertOptions options_;
};

} // namespace pde2d::mesh
