#pragma once
#include "predicates.hpp"
#include "spatial_index.hpp"
#include <map>
#include <queue>
#include <set>

namespace pde2d::discretization {
namespace mesh_detail {
using Edge = std::pair<std::size_t, std::size_t>;
inline Edge edge_key(std::size_t a, std::size_t b) { return std::minmax(a, b); }
inline double distance(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
// One simple closed loop: at least three finite vertices, no zero-length,
// overlapping adjacent or crossing edges.
inline void validate_loop(const std::vector<Point2> &points,
                          const std::vector<std::string> &tags) {
  if (points.size() < 3)
    throw std::invalid_argument("Boundary needs at least three vertices");
  if (!tags.empty() && tags.size() != points.size())
    throw std::invalid_argument("Boundary tag count mismatch");
  const auto n = points.size();
  for (std::size_t i = 0; i < n; ++i) {
    auto a = points[i], c = points[(i + 1) % n], p = points[(i + n - 1) % n];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || distance(a, c) == 0)
      throw std::invalid_argument("Nonfinite or zero-length boundary edge");
    if (predicates::on_segment(p, a, c) || predicates::on_segment(a, c, p))
      throw std::invalid_argument("Overlapping adjacent boundary edges");
  }
  std::vector<Box> boxes;
  for (std::size_t i = 0; i < n; ++i)
    boxes.push_back(Box::segment(points[i], points[(i + 1) % n]));
  BoxIndex index(boxes);
  for (std::size_t i = 0; i < n; ++i) {
    auto a = points[i], c = points[(i + 1) % n];
    for (auto j : index.candidates(boxes[i]))
      if (j > i)
        if ((i + 1) % n != j && (j + 1) % n != i)
          if (predicates::intersects(a, c, points[j], points[(j + 1) % n]))
            throw std::invalid_argument("Boundary self-intersects");
  }
}
// Exact point-in-loop test by winding number: 1 inside, 0 on the loop,
// -1 outside. The loop may run in either direction.
inline int point_in_loop(Point2 p, const std::vector<Point2> &points) {
  int winding = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto a = points[i], b = points[(i + 1) % points.size()];
    if (predicates::on_segment(a, b, p))
      return 0;
    if (a.y <= p.y && b.y > p.y && predicates::orient(a, b, p) > 0)
      ++winding;
    if (a.y > p.y && b.y <= p.y && predicates::orient(a, b, p) < 0)
      --winding;
  }
  return winding != 0 ? 1 : -1;
}
inline std::size_t loop_count(const Boundary &b) { return 1 + b.holes.size(); }
inline const std::vector<Point2> &loop_points(const Boundary &b,
                                              std::size_t loop) {
  return loop == 0 ? b.points : b.holes[loop - 1].points;
}
// All loop vertices, outer loop first, as one point list.
inline std::vector<Point2> boundary_points(const Boundary &boundary) {
  std::vector<Point2> pts = boundary.points;
  for (const auto &hole : boundary.holes)
    pts.insert(pts.end(), hole.points.begin(), hole.points.end());
  return pts;
}
// Every loop is simple (validate_loop); holes lie strictly inside the outer
// loop, strictly outside each other, and no two loops touch. Orientation is
// free: each loop may run either way.
inline void validate_polygon(const Boundary &b) {
  validate_loop(b.points, b.edge_tags);
  if (b.holes.empty())
    return;
  for (const auto &hole : b.holes)
    validate_loop(hole.points, hole.edge_tags);
  struct LoopEdge {
    std::size_t loop;
    Point2 a, b;
  };
  std::vector<LoopEdge> edges;
  std::vector<Box> boxes;
  for (std::size_t k = 0; k < loop_count(b); ++k) {
    const auto &points = loop_points(b, k);
    for (std::size_t i = 0; i < points.size(); ++i) {
      edges.push_back({k, points[i], points[(i + 1) % points.size()]});
      boxes.push_back(Box::segment(edges.back().a, edges.back().b));
    }
  }
  BoxIndex index(boxes);
  for (std::size_t i = 0; i < edges.size(); ++i)
    for (auto j : index.candidates(boxes[i]))
      if (j > i && edges[j].loop != edges[i].loop &&
          predicates::intersects(edges[i].a, edges[i].b, edges[j].a,
                                 edges[j].b))
        throw std::invalid_argument("Boundary loops intersect or touch");
  // Without contacts one vertex decides each containment question.
  for (std::size_t h = 0; h < b.holes.size(); ++h) {
    if (point_in_loop(b.holes[h].points.front(), b.points) <= 0)
      throw std::invalid_argument("Hole is not inside the outer boundary");
    for (std::size_t g = 0; g < b.holes.size(); ++g)
      if (g != h &&
          point_in_loop(b.holes[h].points.front(), b.holes[g].points) > 0)
        throw std::invalid_argument("Holes must not be nested");
  }
}
} // namespace mesh_detail

// The closed loops formed by mesh.boundary_edges, as lists of boundary-edge
// indices. Each loop is listed in traversal order from its lowest edge index,
// continuing from that edge's second vertex. The outer loop (the one through
// the lexicographically least boundary vertex) comes first, the holes follow
// in order of their lowest edge index. Throws when a boundary vertex does not
// have exactly two boundary edges.
inline std::vector<std::vector<std::size_t>>
boundary_loops(const TriangleMesh &mesh) {
  std::map<std::size_t, std::vector<std::size_t>> incident;
  for (std::size_t i = 0; i < mesh.boundary_edges.size(); ++i) {
    auto [a, b] = mesh.boundary_edges[i];
    incident[a].push_back(i);
    incident[b].push_back(i);
  }
  for (const auto &[v, edges] : incident) {
    (void)v;
    if (edges.size() != 2)
      throw std::invalid_argument("Boundary is not a closed loop");
  }
  std::vector<std::vector<std::size_t>> loops;
  std::vector<bool> visited(mesh.boundary_edges.size(), false);
  for (std::size_t start = 0; start < mesh.boundary_edges.size(); ++start) {
    if (visited[start])
      continue;
    auto &loop = loops.emplace_back();
    std::size_t edge = start, vertex = mesh.boundary_edges[start].second;
    while (!visited[edge]) {
      visited[edge] = true;
      loop.push_back(edge);
      const auto &pair = incident.at(vertex);
      edge = pair[0] == edge ? pair[1] : pair[0];
      auto [a, b] = mesh.boundary_edges[edge];
      vertex = a == vertex ? b : a;
    }
  }
  if (loops.size() > 1) {
    auto less = [&](std::size_t u, std::size_t v) {
      auto p = mesh.nodes.at(u), q = mesh.nodes.at(v);
      return p.x < q.x || (p.x == q.x && p.y < q.y);
    };
    std::size_t least = incident.begin()->first, outer = 0;
    for (const auto &[v, edges] : incident) {
      (void)edges;
      if (less(v, least))
        least = v;
    }
    for (std::size_t k = 0; k < loops.size(); ++k)
      for (auto e : loops[k])
        if (mesh.boundary_edges[e].first == least ||
            mesh.boundary_edges[e].second == least)
          outer = k;
    std::rotate(loops.begin(),
                loops.begin() + static_cast<std::ptrdiff_t>(outer),
                loops.begin() + static_cast<std::ptrdiff_t>(outer) + 1);
  }
  return loops;
}

// Strong geometric precondition for external meshes: one connected
// triangulated planar domain whose boundary is one or more closed loops (an
// outer loop and holes; Euler characteristic V - E + F = 2 - loops), no
// unused/duplicate vertices, positive faces, and no crossing edges.
inline void validate_mesh_geometry(const TriangleMesh &mesh) {
  using namespace mesh_detail;
  if (mesh.nodes.empty() || mesh.elements.empty() ||
      mesh.boundary_edges.empty())
    throw std::invalid_argument("Mesh is empty");
  if (!mesh.boundary_tags.empty() &&
      mesh.boundary_tags.size() != mesh.boundary_edges.size())
    throw std::invalid_argument("Boundary tag count mismatch");
  std::set<std::pair<double, double>> coordinates;
  for (auto p : mesh.nodes)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !coordinates.emplace(p.x, p.y).second)
      throw std::invalid_argument("Nonfinite or duplicate mesh vertex");
  std::vector<bool> used(mesh.nodes.size(), false);
  std::map<Edge, std::vector<std::pair<std::size_t, int>>> owners;
  std::set<std::array<std::size_t, 3>> triangles;
  for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
    auto t = mesh.elements[i];
    for (auto v : t) {
      if (v >= mesh.nodes.size())
        throw std::invalid_argument("Invalid triangle index");
      used[v] = true;
    }
    if (predicates::orient(mesh.nodes[t[0]], mesh.nodes[t[1]],
                           mesh.nodes[t[2]]) <= 0)
      throw std::invalid_argument("Triangle must have positive area");
    auto sorted = t;
    std::sort(sorted.begin(), sorted.end());
    if (!triangles.insert(sorted).second)
      throw std::invalid_argument("Duplicate triangle");
    for (int k = 0; k < 3; ++k)
      owners[edge_key(t[k], t[(k + 1) % 3])].push_back(
          {i, t[k] < t[(k + 1) % 3] ? 1 : -1});
  }
  if (std::find(used.begin(), used.end(), false) != used.end())
    throw std::invalid_argument("Mesh contains unused vertices");
  std::set<Edge> boundary;
  std::map<std::size_t, std::vector<std::size_t>> boundary_adj;
  for (auto [a, b] : mesh.boundary_edges) {
    if (a >= mesh.nodes.size() || b >= mesh.nodes.size() || a == b ||
        !boundary.insert(edge_key(a, b)).second)
      throw std::invalid_argument("Invalid or duplicate boundary edge");
    boundary_adj[a].push_back(b);
    boundary_adj[b].push_back(a);
  }
  for (const auto &[v, adj] : boundary_adj) {
    (void)v;
    if (adj.size() != 2)
      throw std::invalid_argument("Boundary is not a closed loop");
  }
  // Every boundary vertex has degree two, so components are closed loops.
  std::size_t loops = 0;
  std::set<std::size_t> visited;
  std::vector<std::size_t> pending;
  for (const auto &[start, adj] : boundary_adj) {
    (void)adj;
    if (visited.contains(start))
      continue;
    ++loops;
    pending = {start};
    while (!pending.empty()) {
      auto v = pending.back();
      pending.pop_back();
      if (visited.insert(v).second)
        for (auto u : boundary_adj.at(v))
          pending.push_back(u);
    }
  }
  std::vector<std::vector<std::size_t>> adjacency(mesh.elements.size());
  for (const auto &[edge, own] : owners) {
    if (own.size() > 2 || (own.size() == 1) != boundary.contains(edge))
      throw std::invalid_argument(
          "Nonmanifold mesh or incorrect boundary records");
    if (own.size() == 2) {
      if (own[0].second == own[1].second)
        throw std::invalid_argument("Inconsistent triangle adjacency");
      adjacency[own[0].first].push_back(own[1].first);
      adjacency[own[1].first].push_back(own[0].first);
    }
  }
  for (auto edge : boundary)
    if (!owners.contains(edge))
      throw std::invalid_argument("Unused boundary edge");
  visited.clear();
  pending = {0};
  while (!pending.empty()) {
    auto v = pending.back();
    pending.pop_back();
    if (visited.insert(v).second)
      for (auto u : adjacency[v])
        pending.push_back(u);
  }
  if (visited.size() != mesh.elements.size())
    throw std::invalid_argument("Disconnected mesh");
  std::vector<Edge> edges;
  std::vector<Box> boxes;
  for (const auto &[e, own] : owners) {
    (void)own;
    edges.push_back(e);
    boxes.push_back(Box::segment(mesh.nodes[e.first], mesh.nodes[e.second]));
  }
  BoxIndex index(boxes);
  for (std::size_t i = 0; i < edges.size(); ++i)
    for (auto j : index.candidates(boxes[i])) {
      if (j <= i)
        continue;
      auto [a, b] = edges[i];
      auto [c, d] = edges[j];
      auto p = mesh.nodes[a], q = mesh.nodes[b], r = mesh.nodes[c],
           s = mesh.nodes[d];
      if (a == c || a == d || b == c || b == d) {
        std::size_t common = (a == c || a == d) ? a : b,
                    u = common == a ? b : a, v = common == c ? d : c;
        if (predicates::on_segment(mesh.nodes[common], mesh.nodes[u],
                                   mesh.nodes[v]) ||
            predicates::on_segment(mesh.nodes[common], mesh.nodes[v],
                                   mesh.nodes[u]))
          throw std::invalid_argument(
              "Overlapping mesh edges / hanging vertex");
      } else if (predicates::intersects(p, q, r, s))
        throw std::invalid_argument("Crossing mesh edges / hanging vertex");
    }
  // A connected planar triangulation with `loops` boundary components (one
  // outer loop, loops - 1 holes) has V - E + F = 2 - loops.
  if (mesh.nodes.size() + mesh.elements.size() + loops != owners.size() + 2)
    throw std::invalid_argument(
        "Mesh is not a planar domain: Euler characteristic disagrees with "
        "its boundary loops");
}
} // namespace pde2d::discretization
