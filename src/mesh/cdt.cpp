#include "pde2d/mesh/cdt.hpp"

#include "delaunay_internal.hpp"
#include "polygon_nodes.hpp"

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

bool properly_cross(Point a, Point b, Point c, Point d) {
  return orient(a, b, c) * orient(a, b, d) < 0 &&
         orient(c, d, a) * orient(c, d, b) < 0;
}

// The edges of `mesh` that cross the open segment a -> b, as node pairs.
std::vector<std::pair<std::size_t, std::size_t>>
crossed_edges(const WorkingMesh &mesh, std::size_t a, std::size_t b) {
  const auto &P = mesh.nodes();
  const Point A = P[a], B = P[b];
  std::vector<std::pair<std::size_t, std::size_t>> result;
  // The triangle around a through which the segment leaves.
  std::size_t right = no_index, left = no_index;
  for (auto t : mesh.triangles_around(a)) {
    const auto &v = mesh.triangles()[t];
    std::size_t k = 0;
    while (v[k] != a)
      ++k;
    std::size_t u = v[(k + 1) % 3], w = v[(k + 2) % 3];
    if (orient(A, B, P[u]) < 0 && orient(A, B, P[w]) > 0) {
      right = u;
      left = w;
      break;
    }
    if (orient(A, B, P[u]) == 0 && orient(A, P[u], B) == 0 &&
        geometry::on_segment(A, B, P[u]))
      throw std::invalid_argument(
          "CDT: a polygon point lies on another polygon edge");
  }
  if (right == no_index)
    throw std::logic_error("CDT: no triangle around the segment start");
  while (true) {
    result.emplace_back(right, left);
    // Across the edge right -> left is the triangle with left -> right.
    auto [t, edge] = mesh.find_edge(left, right);
    if (t == no_index)
      throw std::logic_error("CDT: segment walk left the triangulation");
    std::size_t w = mesh.triangles()[t][static_cast<std::size_t>(edge)];
    if (w == b)
      return result;
    int side = orient(A, B, P[w]);
    if (side == 0)
      throw std::invalid_argument(
          "CDT: a polygon point lies on another polygon edge");
    if (side < 0)
      right = w;
    else
      left = w;
  }
}

std::size_t recover_segment(WorkingMesh &mesh, std::size_t a, std::size_t b) {
  if (mesh.find_edge(a, b).first != no_index ||
      mesh.find_edge(b, a).first != no_index)
    return 0;
  const auto &P = mesh.nodes();
  std::deque<std::pair<std::size_t, std::size_t>> queue;
  for (auto e : crossed_edges(mesh, a, b))
    queue.push_back(e);
  std::size_t flips = 0, guard = 0;
  const std::size_t limit = 1000 + 50 * queue.size() * queue.size();
  while (!queue.empty()) {
    if (++guard > limit)
      throw std::logic_error("CDT: segment recovery did not terminate");
    auto [u, v] = queue.front();
    queue.pop_front();
    auto [t, edge] = mesh.find_edge(u, v);
    if (t == no_index)
      std::tie(t, edge) = mesh.find_edge(v, u);
    if (t == no_index)
      continue;
    if (!mesh.is_flippable(t, edge)) {
      queue.emplace_back(u, v);
      continue;
    }
    std::size_t c = mesh.triangles()[t][static_cast<std::size_t>(edge)];
    std::size_t n = mesh.neighbour(t, edge);
    std::size_t d = 0;
    for (auto x : mesh.triangles()[n])
      if (x != u && x != v)
        d = x;
    mesh.flip(t, edge);
    ++flips;
    if (c != a && c != b && d != a && d != b &&
        properly_cross(P[a], P[b], P[c], P[d]))
      queue.emplace_back(c, d);
  }
  if (mesh.find_edge(a, b).first == no_index &&
      mesh.find_edge(b, a).first == no_index)
    throw std::logic_error("CDT: segment recovery failed");
  return flips;
}

} // namespace

WorkingMesh Cdt::run(const BoundaryPolygon &polygon,
                     StageReport &report) const {
  auto pn = detail::polygon_nodes(polygon);
  const auto &points = pn.nodes;
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (auto p : points) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  const double s = std::max(xmax - xmin, ymax - ymin);
  const Point c{0.5 * (xmin + xmax), 0.5 * (ymin + ymax)};
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
    recovery_flips += recover_segment(scratch, a, b);
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
                   std::move(pn.edges), polygon.region_names, polygon.curves);

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
