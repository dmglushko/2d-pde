#pragma once

#include "pde2d/geometry/point.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace pde2d::mesh {

// A triangle by its three node indices, counterclockwise.
struct Triangle {
  std::array<std::size_t, 3> nodes{};
};

// An edge on the domain boundary, oriented so that the domain is on its
// left. `region` indexes Mesh::region_names.
struct BoundaryEdge {
  std::array<std::size_t, 2> nodes{};
  std::size_t region = 0;
};

// The output mesh: plain arrays, self-contained (no link to a geometry).
// Boundary edges are listed loop by loop, each loop in walking order.
// Interior edges and connected pieces are not stored; compute them when
// needed.
struct Mesh {
  std::vector<geometry::Point> nodes;
  std::vector<Triangle> triangles;
  std::vector<BoundaryEdge> boundary_edges;
  std::vector<std::string> region_names;
};

} // namespace pde2d::mesh
