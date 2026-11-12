#pragma once

// Private to the mesh library: the geometric validity check that the
// WorkingMesh constructor runs after its connectivity checks.

#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace pde2d::mesh::detail {

// The mesh must already pass the connectivity checks: indices in range,
// counterclockwise triangles, every edge used once in each direction at
// most, the edges used once are exactly the boundary edges, and each node
// has at most one outgoing boundary edge (`outgoing_boundary[node]`, or
// no_index).
//
// Then the number of triangles covering a point equals the winding number
// of the boundary edges around it, and the triangles overlap nowhere if
// and only if:
//  1. the triangles around every node turn around it once (interior node)
//     or less than once (boundary node): no fold;
//  2. no two boundary edges cross or touch, except consecutive edges of a
//     loop at their common node;
//  3. no node lies in (or on) a triangle that does not use it, which
//     rules out a piece lying inside another.
// All decisions use exact predicates. A uniform grid limits the pairs
// tested in 2 and 3, so the cost is about linear for meshes whose
// triangles are of similar size. Throws std::invalid_argument naming the
// node, triangle or boundary edges.
void check_geometric_validity(
    const std::vector<geometry::Point> &nodes,
    const std::vector<std::array<std::size_t, 3>> &triangles,
    const std::vector<WorkingBoundaryEdge> &boundary_edges,
    const std::vector<std::size_t> &outgoing_boundary);

} // namespace pde2d::mesh::detail
