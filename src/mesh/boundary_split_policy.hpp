#pragma once

// Private to the mesh library.

#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>

namespace pde2d::mesh::detail {

// How a boundary edge is split at its curve point. This is the library's
// own robustness machinery, not a user choice: the library uses one fixed
// policy (boundary_split_policy()), and changing it means changing the
// library. WorkingMesh::split_boundary_edge calls it and marks the edge
// unsplittable when it fails.
class BoundarySplitPolicy {
public:
  virtual ~BoundarySplitPolicy() = default;
  // Splits boundary edge k at mesh.boundary_split_point(k), or leaves the
  // mesh unchanged and returns split = false with the reason.
  virtual BoundarySplitResult split(WorkingMesh &mesh, std::size_t k) const = 0;
};

// The policy the library uses: ArcLengthMidpointPolicy in
// boundary_split_policy.cpp (see WorkingMesh::split_boundary_edge).
const BoundarySplitPolicy &boundary_split_policy();

} // namespace pde2d::mesh::detail
