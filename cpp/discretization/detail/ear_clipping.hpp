#pragma once
// Private (mesh_detail) part of triangulate.hpp: ear clipping of the sampled
// boundary polygon, the seed triangulation of every meshing algorithm.
#include "../geometry_base.hpp"
#include "../mesh_validation.hpp"
#include "../predicates.hpp"
#include "../spatial_index.hpp"
#include "cdt_topology.hpp"
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace pde2d::discretization::mesh_detail {
inline std::vector<Triangle> ear_clip(const Boundary &boundary) {
  validate_polygon(boundary);
  const auto &pts = boundary.points;
  std::vector<std::size_t> ring(pts.size());
  std::iota(ring.begin(), ring.end(), 0);
  // At the lexicographically least vertex the turn determines loop orientation,
  // including outlines with collinear samples (search through those samples).
  auto least = std::min_element(ring.begin(), ring.end(), [&](auto a, auto b) {
    return std::tie(pts[a].x, pts[a].y) < std::tie(pts[b].x, pts[b].y);
  });
  auto k = static_cast<std::size_t>(least - ring.begin());
  int winding = 0;
  for (std::size_t offset = 1; offset < ring.size() && !winding; ++offset)
    winding =
        predicates::orient(pts[ring[(k + ring.size() - offset) % ring.size()]],
                           pts[*least], pts[ring[(k + offset) % ring.size()]]);
  if (!winding)
    throw std::invalid_argument("Zero-area polygon");
  if (winding < 0)
    std::reverse(ring.begin(), ring.end());
  std::vector<Box> point_boxes;
  for (auto p : pts)
    point_boxes.push_back(Box::segment(p, p));
  BoxIndex point_index(std::move(point_boxes));
  std::vector<bool> active(pts.size(), true);
  std::vector<Triangle> triangles;
  while (ring.size() > 3) {
    bool clipped = false;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      auto a = ring[(i + ring.size() - 1) % ring.size()], b = ring[i],
           c = ring[(i + 1) % ring.size()];
      if (predicates::orient(pts[a], pts[b], pts[c]) <= 0)
        continue;
      bool contains = false;
      auto box = Box::segment(pts[a], pts[b]);
      box = {std::min(box.xmin, pts[c].x), std::max(box.xmax, pts[c].x),
             std::min(box.ymin, pts[c].y), std::max(box.ymax, pts[c].y)};
      for (auto v : point_index.candidates(box))
        if (active[v] && v != a && v != b && v != c &&
            in_triangle(pts[v], pts[a], pts[b], pts[c])) {
          contains = true;
          break;
        }
      if (contains)
        continue;
      triangles.push_back({a, b, c});
      active[b] = false;
      ring.erase(ring.begin() + i);
      clipped = true;
      break;
    }
    if (!clipped)
      throw std::runtime_error("Could not find a nondegenerate ear");
  }
  triangles.push_back(ccw({ring[0], ring[1], ring[2]}, pts));
  return triangles;
}
inline TriangleMesh seed_mesh(const Boundary &boundary) {
  TriangleMesh m;
  m.nodes = boundary.points;
  m.elements = ear_clip(boundary);
  m.boundary_tags = boundary.edge_tags;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    m.boundary_edges.emplace_back(i, (i + 1) % m.nodes.size());
  return m;
}
} // namespace pde2d::discretization::mesh_detail
