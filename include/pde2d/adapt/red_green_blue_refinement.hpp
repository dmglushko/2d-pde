#pragma once

#include "pde2d/adapt/refinement.hpp"

namespace pde2d::adapt {

// Regular (red) refinement with bisection closure: red-green-blue
// refinement compatible with newest vertex bisection.
//
// Edges are marked exactly as in NewestVertexBisection (the three edges of
// every marked triangle, then the closure: a triangle with a marked edge
// has its refinement edge, opposite vertex 0, marked too). Then:
//  - three marked edges: red. The triangle (v0, v1, v2), with midpoints
//    m0, m1, m2 of the edges opposite v0, v1, v2, becomes
//      (v0, m2, m1), (m2, v1, m0), (m1, m0, v2) and (m0, m1, m2).
//    Each is the parent scaled by 1/2 (the last one by -1/2, a half turn),
//    and its vertices are listed in the order of the parent vertices they
//    are images of, so each child's refinement edge is the image of the
//    parent's.
//  - one marked edge (the refinement edge): green, one bisection, as in
//    NVB.
//  - two marked edges: blue, two bisections, as in NVB.
//
// Why repeated refinement does not degenerate, even though green and blue
// triangles are kept (not removed and redone) in later cycles: green and
// blue are NVB bisections, and a red child is similar to its parent with
// the refinement edge in the corresponding place, so it is refined later
// exactly like a copy of its parent. By induction every triangle ever
// made is similar to a triangle that NVB alone makes from an initial
// triangle, and NVB makes at most four similarity classes per initial
// triangle. The smallest angle is therefore bounded below by a constant
// that depends only on the initial mesh (and its refinement edges).
//
// Near a curved boundary this does not hold, and red refinement is
// worse there than NVB: a triangle with two linked boundary edges on a
// smooth curve (an "ear" a, b, c with a-b and b-c on the boundary) has as
// red child at b the triangle of b and the two new curve points, three
// consecutive points of the curve, whose small angles are about half the
// turning angle between them; it halves with every red refinement. NVB
// joins those curve points to the midpoint of a-c instead. An initial
// mesh without such ears on curved parts (for example from Ruppert, whose
// angle bound rules them out) keeps the angles bounded in practice.
class RedGreenBlueRefinement final : public RefinementMethod {
public:
  std::string name() const override { return "red_green_blue"; }
  [[nodiscard]] RefinementResult
  refine(const mesh::Mesh &mesh,
         std::span<const std::size_t> marked) const override;
};

} // namespace pde2d::adapt
