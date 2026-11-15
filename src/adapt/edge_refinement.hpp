#pragma once

// Private to the adapt library: the refinement shared by
// NewestVertexBisection and RedGreenBlueRefinement. The two differ only in
// how a triangle with all three edges marked is divided.

#include "pde2d/adapt/refinement.hpp"

#include <cstddef>
#include <span>
#include <string>

namespace pde2d::adapt::detail {

// How a triangle whose three edges are all marked is divided.
enum class FullRefinement {
  bisections, // NVB: bisect, then bisect both children
  red         // four children similar to the parent
};

// Marks the three edges of every marked triangle, closes the marking
// (a triangle with a marked edge gets its refinement edge, opposite vertex
// 0, marked), gives every marked edge a node (the midpoint, or the curve
// point of a linked boundary edge, from WorkingMesh::boundary_split_point)
// and divides each triangle by the pattern of its marked edges. `name` is
// the method's name, for the report and error messages. See
// RefinementMethod::refine for the result and the errors.
RefinementResult refine_marked_edges(const mesh::Mesh &mesh,
                                     std::span<const std::size_t> marked,
                                     const std::string &name,
                                     FullRefinement full);

} // namespace pde2d::adapt::detail
