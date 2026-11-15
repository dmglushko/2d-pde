#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct RedGreenBlueRefinementOptions {
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Regular (red) refinement with bisection closure: red-green-blue
// refinement compatible with newest vertex bisection.
//
// refine(): all three edges of every marked triangle are split, with the
// closure of NewestVertexBisection (a triangle with a split edge is first
// bisected at its refinement edge, opposite vertex 0). Then:
//  - three split edges: red. The triangle (v0, v1, v2), with midpoints
//    m0, m1, m2 of the edges opposite v0, v1, v2, becomes
//      (v0, m2, m1), (m2, v1, m0), (m1, m0, v2) and (m0, m1, m2).
//    Each is the parent scaled by 1/2 (the last one by -1/2, a half turn),
//    and its vertices are listed in the order of the parent vertices they
//    are images of, so each child's refinement edge is the image of the
//    parent's. (It is made from the bisection pattern by one flip; where a
//    curve point makes that flip impossible the triangle keeps the
//    bisection pattern, counted in "red_not_possible" with a message.)
//  - one split edge (the refinement edge): green, one bisection, as in
//    NVB.
//  - two split edges: blue, two bisections, as in NVB.
// The parents are kept as in NewestVertexBisection.
//
// Why repeated refinement does not degenerate, even though green and blue
// triangles are kept (not removed and redone) in later cycles: green and
// blue are NVB bisections, and a red child is similar to its parent with
// the refinement edge in the corresponding place, so it is refined later
// exactly like a copy of its parent. By induction every triangle ever
// made is similar to a triangle that NVB alone makes from an initial
// triangle, and NVB makes at most four similarity classes per initial
// triangle (with the refinement edges set as NVB needs them).
//
// Near a curved boundary this does not hold, and red refinement is worse
// there than NVB: a triangle with two linked boundary edges on a smooth
// curve (an "ear" a, b, c with a-b and b-c on the boundary) has as red
// child at b the triangle of b and the two new curve points, three
// consecutive points of the curve, whose small angles are about half the
// turning angle between them; it halves with every red refinement. The
// Remesher's mesh properties show it (the warning angle in the pipeline
// report). An initial mesh without such ears on curved parts (for example
// from Ruppert) keeps the angles bounded in practice.
//
// run(): while a triangle has an edge longer than max_edge_length
// (resolved against domain_size(mesh)), refine() all such triangles.
//
// Local improvers run after each refine() step, after the red flips.
// Metrics as LongestEdgeBisection, plus "red_triangles" and
// "red_not_possible".
class RedGreenBlueRefinement final : public Refiner {
public:
  explicit RedGreenBlueRefinement(RedGreenBlueRefinementOptions options = {});
  const RedGreenBlueRefinementOptions &options() const { return options_; }

  std::string name() const override { return "red_green_blue"; }
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
  RedGreenBlueRefinementOptions options_;
};

} // namespace pde2d::mesh
