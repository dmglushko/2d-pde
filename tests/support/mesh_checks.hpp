#pragma once

// Independent checks of output meshes. They use plain (long) double
// arithmetic and std::map, not the library's predicates or data
// structures, so they do not repeat the code under test.

#include "support/check.hpp"

#include "pde2d/mesh/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <utility>

namespace pde2d::test {

using pde2d::geometry::Point;
using pde2d::mesh::Mesh;

inline double triangle_area(Point a, Point b, Point c) {
  return 0.5 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

inline double mesh_area(const Mesh &m) {
  double area = 0;
  for (const auto &t : m.triangles)
    area += triangle_area(m.nodes[t.nodes[0]], m.nodes[t.nodes[1]],
                          m.nodes[t.nodes[2]]);
  return area;
}

// Area enclosed by the boundary edges (holes count negatively because
// their edges run clockwise).
inline double boundary_area(const Mesh &m) {
  double twice = 0;
  for (const auto &e : m.boundary_edges) {
    Point a = m.nodes[e.nodes[0]], b = m.nodes[e.nodes[1]];
    twice += a.x * b.y - a.y * b.x;
  }
  return twice / 2;
}

// Positive orientation, conformity, and: the directed edges used by exactly
// one triangle are exactly the boundary edges.
inline void check_valid_mesh(const Mesh &m) {
  check(!m.triangles.empty(), "mesh has triangles");
  std::map<std::pair<std::size_t, std::size_t>, int> directed;
  for (const auto &t : m.triangles) {
    for (auto v : t.nodes)
      check(v < m.nodes.size(), "node index in range");
    check(triangle_area(m.nodes[t.nodes[0]], m.nodes[t.nodes[1]],
                        m.nodes[t.nodes[2]]) > 0,
          "triangle is counterclockwise and not degenerate");
    for (int i = 0; i < 3; ++i)
      ++directed[{t.nodes[i], t.nodes[(i + 1) % 3]}];
  }
  std::set<std::pair<std::size_t, std::size_t>> once;
  for (auto [edge, count] : directed) {
    check(count == 1, "no directed edge is used twice");
    auto [a, b] = edge;
    if (!directed.count({b, a}))
      once.insert(edge);
  }
  std::set<std::pair<std::size_t, std::size_t>> boundary;
  for (const auto &e : m.boundary_edges) {
    check(e.region < m.region_names.size(), "region id in range");
    check(boundary.insert({e.nodes[0], e.nodes[1]}).second,
          "boundary edge listed once");
  }
  check(once == boundary,
        "edges used by one triangle are exactly the boundary edges");
  std::vector<bool> used(m.nodes.size(), false);
  for (const auto &t : m.triangles)
    for (auto v : t.nodes)
      used[v] = true;
  check(std::all_of(used.begin(), used.end(), [](bool b) { return b; }),
        "every node is used");
}

inline void check_area(const Mesh &m, double expected, double tolerance) {
  near(mesh_area(m), boundary_area(m), 1e-12 * std::abs(boundary_area(m)),
       "triangle areas add up to the area inside the boundary");
  near(mesh_area(m), expected, tolerance, "mesh area");
}

// d strictly inside the circumcircle of counterclockwise (a, b, c), with a
// relative tolerance so cocircular points are accepted.
inline bool strictly_in_circumcircle(Point a, Point b, Point c, Point d) {
  long double ax = a.x - d.x, ay = a.y - d.y, bx = b.x - d.x, by = b.y - d.y,
              cx = c.x - d.x, cy = c.y - d.y;
  long double al = ax * ax + ay * ay, bl = bx * bx + by * by,
              cl = cx * cx + cy * cy;
  long double det = al * (bx * cy - by * cx) - bl * (ax * cy - ay * cx) +
                    cl * (ax * by - ay * bx);
  long double scale = al * (std::abs(bx * cy) + std::abs(by * cx)) +
                      bl * (std::abs(ax * cy) + std::abs(ay * cx)) +
                      cl * (std::abs(ax * by) + std::abs(ay * bx));
  return det > 1e-12L * scale;
}

// Every interior edge is locally Delaunay (constrained Delaunay with the
// boundary edges as constraints).
inline bool is_delaunay(const Mesh &m) {
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> owner;
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    for (int i = 0; i < 3; ++i)
      owner[{m.triangles[t].nodes[i], m.triangles[t].nodes[(i + 1) % 3]}] = t;
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    const auto &v = m.triangles[t].nodes;
    for (int i = 0; i < 3; ++i) {
      auto it = owner.find({v[(i + 1) % 3], v[i]});
      if (it == owner.end())
        continue;
      const auto &w = m.triangles[it->second].nodes;
      std::size_t d = w[0] + w[1] + w[2] - v[i] - v[(i + 1) % 3];
      if (strictly_in_circumcircle(m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]],
                                   m.nodes[d]))
        return false;
    }
  }
  return true;
}

inline double min_angle_degrees(const Mesh &m) {
  double smallest = 180;
  for (const auto &t : m.triangles)
    for (int i = 0; i < 3; ++i) {
      Point a = m.nodes[t.nodes[i]], b = m.nodes[t.nodes[(i + 1) % 3]],
            c = m.nodes[t.nodes[(i + 2) % 3]];
      double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - a.x, vy = c.y - a.y;
      double angle = std::acos((ux * vx + uy * vy) /
                               (std::hypot(ux, uy) * std::hypot(vx, vy)));
      smallest = std::min(smallest, angle * 180 / std::numbers::pi);
    }
  return smallest;
}

inline double longest_edge(const Mesh &m) {
  double longest = 0;
  for (const auto &t : m.triangles)
    for (int i = 0; i < 3; ++i) {
      Point a = m.nodes[t.nodes[i]], b = m.nodes[t.nodes[(i + 1) % 3]];
      longest = std::max(longest, std::hypot(a.x - b.x, a.y - b.y));
    }
  return longest;
}

inline std::set<std::size_t> boundary_nodes(const Mesh &m) {
  std::set<std::size_t> nodes;
  for (const auto &e : m.boundary_edges) {
    nodes.insert(e.nodes[0]);
    nodes.insert(e.nodes[1]);
  }
  return nodes;
}

inline std::size_t region_id(const Mesh &m, const std::string &name) {
  auto it = std::find(m.region_names.begin(), m.region_names.end(), name);
  check(it != m.region_names.end(), "region exists: " + name);
  return static_cast<std::size_t>(it - m.region_names.begin());
}

inline bool same_mesh(const Mesh &a, const Mesh &b) {
  if (a.nodes.size() != b.nodes.size() ||
      a.triangles.size() != b.triangles.size() ||
      a.boundary_edges.size() != b.boundary_edges.size() ||
      a.region_names != b.region_names)
    return false;
  for (std::size_t i = 0; i < a.nodes.size(); ++i)
    if (!(a.nodes[i] == b.nodes[i]))
      return false;
  for (std::size_t i = 0; i < a.triangles.size(); ++i)
    if (a.triangles[i].nodes != b.triangles[i].nodes)
      return false;
  for (std::size_t i = 0; i < a.boundary_edges.size(); ++i)
    if (a.boundary_edges[i].nodes != b.boundary_edges[i].nodes ||
        a.boundary_edges[i].region != b.boundary_edges[i].region)
      return false;
  return true;
}

} // namespace pde2d::test
