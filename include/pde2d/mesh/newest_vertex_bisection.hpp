#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct NewestVertexBisectionOptions {
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Newest vertex bisection (NVB). The refinement edge of a triangle is the
// edge opposite its vertex 0 (the convention in mesh.hpp). Bisecting
// triangle (v0, v1, v2) joins v0 to the midpoint m of its refinement edge
// v1-v2, giving the children (m, v0, v1) and (m, v2, v0): m, the newest
// vertex, comes first, so each child's refinement edge is one of the
// parent's other two edges.
//
// refine(): each marked triangle is bisected at its refinement edge (once:
// two children). A neighbour across a split edge whose refinement edge is
// another one is bisected there first, then its child at the shared edge
// (the closure: no node hangs). So a triangle is divided into 2, 3 or 4
// children, always by the bisection pattern, and the mesh stays
// conforming. Every new node is the midpoint of an input edge (the curve
// point of a linked boundary edge) and every new triangle lies in an input
// triangle, so the parents are kept, unless the boundary split policy has
// to re-triangulate.
//
// Shape guarantee, away from curved boundaries: the triangles made by
// repeated refinement fall into at most four similarity classes per
// triangle of the mesh whose refinement edges were set (for example by
// set_longest_edges_as_refinement_edges), so their angles stay bounded
// below. Refinement edges that wait for each other in a cycle (possible
// only when they were not set that way, for example after flips) are
// still refined conformingly, but those triangles leave the pattern (the
// metric "incompatible_splits" and a report message say so).
//
// run(): while a triangle has an edge longer than max_edge_length
// (resolved against domain_size(mesh)), refine() all such triangles.
//
// Local improvers (it tolerates flips and node moves, though flips change
// refinement edges) run after each refine() step on the triangles it
// touched. Metrics as LongestEdgeBisection.
class NewestVertexBisection final : public Refiner {
public:
  explicit NewestVertexBisection(NewestVertexBisectionOptions options = {});
  const NewestVertexBisectionOptions &options() const { return options_; }

  std::string name() const override { return "newest_vertex_bisection"; }
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
  NewestVertexBisectionOptions options_;
};

} // namespace pde2d::mesh
