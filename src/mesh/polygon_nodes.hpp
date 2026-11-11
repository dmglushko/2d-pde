#pragma once

// Private to the mesh library.

#include "pde2d/mesh/boundary_polygon.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <vector>

namespace pde2d::mesh::detail {

// The polygon's points numbered loop after loop (point i of loop l is node
// offset[l] + i) and its edges as working-mesh boundary edges.
struct PolygonNodes {
  std::vector<geometry::Point> nodes;
  std::vector<WorkingBoundaryEdge> edges;
  std::vector<std::size_t> offset;
};

inline PolygonNodes polygon_nodes(const BoundaryPolygon &polygon) {
  PolygonNodes result;
  for (const auto &loop : polygon.loops) {
    std::size_t base = result.nodes.size();
    result.offset.push_back(base);
    const std::size_t n = loop.points.size();
    for (std::size_t i = 0; i < n; ++i) {
      result.nodes.push_back(loop.points[i]);
      result.edges.push_back({{base + i, base + (i + 1) % n},
                              loop.edges[i].region,
                              loop.edges[i].link});
    }
  }
  return result;
}

} // namespace pde2d::mesh::detail
