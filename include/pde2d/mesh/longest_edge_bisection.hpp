#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct LongestEdgeBisectionOptions {
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Rivara's longest-edge bisection: while a triangle has an edge longer
// than max_edge_length, follow the longest-edge propagation path from it
// (the neighbour across the longest edge, repeatedly) to an edge that is
// the longest edge of both its triangles, or a boundary edge, and split
// that edge at its midpoint (a boundary edge with a curve link at the
// curve point). Keeps the mesh conforming without other changes. After
// every split it calls the local improvers it is given (it tolerates
// flips and node moves) on the touched triangles.
// Metrics: "splits", "boundary_splits", "local_improver_changes" (the sum
// of the counts the local improvers return) and, per local improver,
// "local_improver_changes.<name>".
class LongestEdgeBisection final : public Refiner {
public:
  explicit LongestEdgeBisection(LongestEdgeBisectionOptions options = {});
  const LongestEdgeBisectionOptions &options() const { return options_; }

  std::string name() const override { return "longest_edge_bisection"; }
  RefinerCapabilities capabilities() const override {
    return {.tolerates_flips = true,
            .tolerates_node_moves = true,
            .maintains_delaunay = false};
  }
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           std::span<const LocalImprover *const> local_improvers,
           StageReport &report) const override;

private:
  LongestEdgeBisectionOptions options_;
};

} // namespace pde2d::mesh
