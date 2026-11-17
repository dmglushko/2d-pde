#pragma once

// Private to the mesh library: the longest edge of a triangle in the one
// order of all edges that the bisection core and
// set_longest_edges_as_refinement_edges share, and making it the
// refinement edge of a working mesh's triangles (the Remesher's last step
// before its output mesh).

#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace pde2d::mesh::detail {

// The local index i (0, 1 or 2) of the longest edge of the triangle with
// vertices v, the edge opposite vertex i. Edges are ordered by squared
// length, then by their node indices as a (smaller, larger) pair: a strict
// order of all edges, which the two triangles of an edge agree on, so the
// triangles waiting for each other's refinement edges cannot form a cycle.
int longest_local_edge(const std::array<std::size_t, 3> &v,
                       const std::vector<geometry::Point> &nodes);

// Which triangles set_longest_edges_as_refinement_edges rotates.
enum class RefinementEdgeScope {
  // every live triangle (Remesher::run, the bisection refiners' run)
  all,
  // only the live triangles whose vertex order an edit chose: not the
  // order the WorkingMesh constructor received or set_refinement_edge last
  // gave them (Remesher::refine: triangles made or changed by flips, point
  // insertions and re-triangulating boundary splits, but not those a
  // bisection made, nor untouched ones)
  edited_order
};

// Makes the longest edge (longest_local_edge) the refinement edge of the
// triangles in `scope`, with WorkingMesh::set_refinement_edge (a rotation:
// counterclockwise, the same triangle). Returns how many triangles changed
// their vertex 0.
std::size_t set_longest_edges_as_refinement_edges(WorkingMesh &mesh,
                                                  RefinementEdgeScope scope);

// Reads what WorkingMesh records for this (a friend of WorkingMesh).
struct RefinementEdgeRecord {
  // True when triangle t's vertices are exactly as the constructor
  // received them or set_refinement_edge last left them.
  static bool order_is_as_set(const WorkingMesh &mesh, std::size_t t);
};

} // namespace pde2d::mesh::detail
