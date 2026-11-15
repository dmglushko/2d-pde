#pragma once

#include "pde2d/adapt/refinement.hpp"

namespace pde2d::adapt {

// Newest vertex bisection (NVB). The refinement edge of a triangle is the
// edge opposite its vertex 0 (see refinement.hpp). Bisecting triangle
// (v0, v1, v2) joins v0 to the midpoint m of its refinement edge v1-v2,
// giving the children (m, v0, v1) and (m, v2, v0): m, the newest vertex,
// comes first, so each child's refinement edge is one of the parent's
// other two edges.
//
// One call, by marking edges:
//  1. the three edges of every marked triangle are marked;
//  2. closure: a triangle with a marked edge also gets its refinement
//     edge marked, repeated until nothing changes (it ends: edges are only
//     ever added);
//  3. every marked edge gets a new node, and each triangle is bisected
//     while its refinement edge is marked: with one marked edge (its
//     refinement edge) into 2 triangles, with two into 3, with all three
//     into 4 (bisected, then both children bisected).
// So every marked triangle has all its edges halved (4 children, like red
// refinement) and the mesh stays conforming, because an edge is split for
// both of its triangles or for neither.
class NewestVertexBisection final : public RefinementMethod {
public:
  std::string name() const override { return "newest_vertex_bisection"; }
  [[nodiscard]] RefinementResult
  refine(const mesh::Mesh &mesh,
         std::span<const std::size_t> marked) const override;
};

} // namespace pde2d::adapt
