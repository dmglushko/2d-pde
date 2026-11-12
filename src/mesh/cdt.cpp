#include "pde2d/mesh/cdt.hpp"

#include "delaunay_internal.hpp"
#include "edge_recovery.hpp"
#include "polygon_nodes.hpp"

#include "pde2d/geometry/bounding_box.hpp"
#include "pde2d/geometry/predicates.hpp"
#include "pde2d/mesh/delaunay.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <stdexcept>
#include <utility>

namespace pde2d::mesh {

using geometry::orient;
using geometry::Point;

namespace {

// Nodes 0, 1, 2 of the scratch mesh are the enclosing triangle; polygon
// point k is node k + super_count.
constexpr std::size_t super_count = 3;

} // namespace

WorkingMesh Cdt::run(const BoundaryPolygon &polygon,
                     StageReport &report) const {
  auto pn = detail::polygon_nodes(polygon);
  const auto &points = pn.nodes;
  const geometry::BoundingBox box = geometry::bounding_box(points);
  const double s = std::max(width(box), height(box));
  const Point c = midpoint(box.min, box.max);
  // Exact predicates make the size of the enclosing triangle harmless.
  WorkingMesh scratch({c + Point{0, 40 * s}, c + Point{-40 * s, -20 * s},
                       c + Point{40 * s, -20 * s}},
                      {{0, 1, 2}},
                      {{{0, 1}, 0, std::nullopt},
                       {{1, 2}, 0, std::nullopt},
                       {{2, 0}, 0, std::nullopt}},
                      {"enclosing"});

  // 1. Delaunay triangulation of all points.
  std::size_t insertion_flips = 0, last = 0;
  for (std::size_t k = 0; k < points.size(); ++k) {
    Location loc = scratch.locate(points[k], last);
    std::vector<std::size_t> created;
    if (loc.kind == Location::Kind::inside) {
      auto t = scratch.insert_in_triangle(loc.triangle, points[k]);
      created.assign(t.begin(), t.end());
    } else if (loc.kind == Location::Kind::on_edge &&
               scratch.neighbour(loc.triangle, loc.index) != no_index) {
      auto t = scratch.split_interior_edge(loc.triangle, loc.index, points[k]);
      created.assign(t.begin(), t.end());
    } else if (loc.kind == Location::Kind::on_node) {
      throw std::invalid_argument("CDT: the polygon repeats a point");
    } else {
      throw std::logic_error("CDT: point outside the enclosing triangle");
    }
    insertion_flips += detail::lawson(scratch, created, nullptr);
    last = created.front();
  }

  // 2. Recover the polygon edges.
  std::size_t recovery_flips = 0;
  std::set<std::pair<std::size_t, std::size_t>> constraints;
  for (const auto &e : pn.edges) {
    std::size_t a = e.nodes[0] + super_count, b = e.nodes[1] + super_count;
    recovery_flips += detail::recover_edge(scratch, a, b);
    constraints.insert({std::min(a, b), std::max(a, b)});
  }

  // 3. Keep the triangles on the left of the polygon edges, spreading
  //    across edges that are not polygon edges.
  const auto &T = scratch.triangles();
  std::vector<bool> inside(T.size(), false);
  std::vector<std::size_t> stack;
  for (const auto &e : pn.edges) {
    auto [t, edge] =
        scratch.find_edge(e.nodes[0] + super_count, e.nodes[1] + super_count);
    if (t == no_index)
      throw std::logic_error("CDT: a polygon edge is missing");
    if (!inside[t]) {
      inside[t] = true;
      stack.push_back(t);
    }
  }
  while (!stack.empty()) {
    std::size_t t = stack.back();
    stack.pop_back();
    for (int i = 0; i < 3; ++i) {
      std::size_t a = T[t][static_cast<std::size_t>((i + 1) % 3)],
                  b = T[t][static_cast<std::size_t>((i + 2) % 3)];
      if (constraints.count({std::min(a, b), std::max(a, b)}))
        continue;
      std::size_t u = scratch.neighbour(t, i);
      if (u == no_index)
        throw std::logic_error("CDT: the domain reaches the enclosing "
                               "triangle; the polygon is not closed");
      if (!inside[u]) {
        inside[u] = true;
        stack.push_back(u);
      }
    }
  }
  std::vector<WorkingMesh::TriangleNodes> triangles;
  for (std::size_t t = 0; t < T.size(); ++t) {
    if (!inside[t])
      continue;
    WorkingMesh::TriangleNodes v = T[t];
    for (auto &x : v) {
      if (x < super_count)
        throw std::logic_error("CDT: domain triangle uses the enclosing "
                               "triangle");
      x -= super_count;
    }
    triangles.push_back(v);
  }
  WorkingMesh mesh(std::move(pn.nodes), std::move(triangles),
                   std::move(pn.edges), polygon.region_names, polygon.chains);

  // 4. Constrained Delaunay: the polygon edges are now boundary edges and
  //    are never flipped.
  std::size_t final_flips = flip_to_delaunay(mesh);
  report.metrics["triangles"] = static_cast<double>(mesh.triangles().size());
  report.metrics["insertion_flips"] = static_cast<double>(insertion_flips);
  report.metrics["recovery_flips"] = static_cast<double>(recovery_flips);
  report.metrics["final_flips"] = static_cast<double>(final_flips);
  return mesh;
}

} // namespace pde2d::mesh
