#pragma once
#include "predicates.hpp"
#include "spatial_index.hpp"
#include <map>
#include <queue>
#include <set>

namespace discretization {
namespace mesh_detail {
using Edge = std::pair<std::size_t, std::size_t>;
inline Edge edge_key(std::size_t a, std::size_t b) { return std::minmax(a, b); }
inline double distance(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline void validate_polygon(const Boundary &b) {
  if (b.points.size() < 3)
    throw std::invalid_argument("Boundary needs at least three vertices");
  if (!b.edge_tags.empty() && b.edge_tags.size() != b.points.size())
    throw std::invalid_argument("Boundary tag count mismatch");
  const auto n = b.points.size();
  for (std::size_t i = 0; i < n; ++i) {
    auto a = b.points[i], c = b.points[(i + 1) % n],
         p = b.points[(i + n - 1) % n];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || distance(a, c) == 0)
      throw std::invalid_argument("Nonfinite or zero-length boundary edge");
    if (predicates::on_segment(p, a, c) || predicates::on_segment(a, c, p))
      throw std::invalid_argument("Overlapping adjacent boundary edges");
  }
  std::vector<Box> boxes;
  for (std::size_t i = 0; i < n; ++i)
    boxes.push_back(Box::segment(b.points[i], b.points[(i + 1) % n]));
  BoxIndex index(boxes);
  for (std::size_t i = 0; i < n; ++i) {
    auto a = b.points[i], c = b.points[(i + 1) % n];
    for (auto j : index.candidates(boxes[i]))
      if (j > i)
        if ((i + 1) % n != j && (j + 1) % n != i)
          if (predicates::intersects(a, c, b.points[j], b.points[(j + 1) % n]))
            throw std::invalid_argument("Boundary self-intersects");
  }
}
} // namespace mesh_detail

// Strong geometric precondition for external meshes: one connected triangulated
// disk, no unused/duplicate vertices, positive faces, and no crossing edges.
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
  std::set<std::size_t> visited;
  std::vector<std::size_t> pending{boundary_adj.begin()->first};
  while (!pending.empty()) {
    auto v = pending.back();
    pending.pop_back();
    if (visited.insert(v).second)
      for (auto u : boundary_adj.at(v))
        pending.push_back(u);
  }
  if (visited.size() != boundary_adj.size())
    throw std::invalid_argument("Only one boundary loop is supported");
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
}
} // namespace discretization
