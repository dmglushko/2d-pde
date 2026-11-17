#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct LongestEdgeBisectionOptions {
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Longest-edge bisection (Rivara). It shares the edge bisection of
// NewestVertexBisection and differs in one rule: a triangle that this
// refinement has not bisected yet is bisected at its longest edge (ties
// broken by node indices), not at the edge opposite its vertex 0. So it
// needs no refinement edges in the mesh.
//
// refine(): each marked triangle has its longest edge split. A neighbour
// across a split edge whose longest edge is another one has that edge
// split first (and so on: the longest-edge propagation path), then is
// bisected again at the shared edge; the mesh stays conforming without
// other changes. Every new node is the midpoint of an input edge (the
// curve point of a linked boundary edge) and every new triangle lies in
// an input triangle, so the parents are kept, unless the boundary split
// policy has to re-triangulate. The new node is vertex 0 of each triangle
// a split makes.
//
// run(): while a triangle has an edge longer than max_edge_length
// (resolved against domain_size(mesh)), refine() all such triangles.
//
// Local improvers (it tolerates flips and node moves) run after every
// split on the triangles it made. A flip changes which triangles wait for
// which, so with flipping local improvers a marked triangle may end up
// refined differently (the parents are then dropped anyway).
// Metrics of both: "splits", "boundary_splits", "retriangulated_splits",
// "deleted_nodes", "unsplittable_edges", "incompatible_splits" (always 0
// here: longest edges never wait in a cycle), "local_improver_changes"
// (the sum of the counts the local improvers return) and, per local
// improver, "local_improver_changes.<name>". run() adds "steps",
// "domain_size" and "max_edge_length" (resolved); refine() adds "marked"
// and "refined_by_closure" (input triangles refined though not marked).
class LongestEdgeBisection final : public Refiner {
public:
  explicit LongestEdgeBisection(LongestEdgeBisectionOptions options = {});
  const LongestEdgeBisectionOptions &options() const { return options_; }

  std::string name() const override { return "longest_edge_bisection"; }
  RefinerCapabilities capabilities() const override {
    return {.tolerates_flips = true,
            .tolerates_node_moves = true,
            .maintains_delaunay = false,
            .keeps_parents = true};
  }
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           std::span<const LocalImprover *const> local_improvers,
           StageReport &report) const override;
  std::optional<Parents>
  refine(WorkingMesh &mesh, std::span<const std::size_t> marked_triangles,
         std::span<const LocalImprover *const> local_improvers,
         StageReport &report) const override;

private:
  LongestEdgeBisectionOptions options_;
};

} // namespace pde2d::mesh
