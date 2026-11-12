#pragma once

// Private to the mesh library.

#include "pde2d/mesh/working_mesh.hpp"

#include <array>
#include <cstddef>

namespace pde2d::mesh::detail {

// The steps of the boundary split policy (boundary_split_policy.cpp). They
// edit the internals of the WorkingMesh, which names this struct a friend;
// only WorkingMesh::split_boundary_edge is public. Each step keeps the mesh
// valid or throws without changing it.
struct BoundarySplitSteps {
  // Step (a): connects the split point to the vertex opposite boundary
  // edge k. Throws std::runtime_error if the point would invert a
  // triangle. Returns the two triangles on the new boundary edges.
  static std::array<std::size_t, 2> split_directly(WorkingMesh &mesh,
                                                   std::size_t k);
  // Adds the triangle (a, p, b) on boundary edge k = a -> b, where p is
  // its split point and must lie strictly on the right of the edge
  // (outside the mesh; std::invalid_argument otherwise). The boundary
  // becomes a -> p -> b. It does not check that the new triangle stays
  // clear of distant parts of the mesh. Returns the old and the new
  // triangle.
  static std::array<std::size_t, 2> add_cap(WorkingMesh &mesh, std::size_t k);
  // Makes a -> q -> b the boundary in place of boundary edge k = a -> b
  // and deletes everything between them: the triangles in the region
  // bounded by a -> b, b -> q and q -> a, and the nodes inside it. q must
  // be an interior node at boundary_split_point(k), joined to a and b by
  // edges, and the region must contain no other boundary edge or boundary
  // node (std::invalid_argument otherwise). Returns the live triangles on
  // the two new boundary edges; the counts of deleted nodes and triangles
  // go to the optional out-parameters.
  static std::array<std::size_t, 2>
  cut_off(WorkingMesh &mesh, std::size_t k, std::size_t q,
          std::size_t *deleted_nodes = nullptr,
          std::size_t *deleted_triangles = nullptr);
};

} // namespace pde2d::mesh::detail
