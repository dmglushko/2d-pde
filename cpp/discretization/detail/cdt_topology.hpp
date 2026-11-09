#pragma once
// Private (mesh_detail) part of triangulate.hpp: incremental constrained
// Delaunay topology. Topology tracks edge ownership, constrained boundary
// edges, pending legalization work and a lazily invalidated longest-edge heap
// for size refinement; cdt_verified() is the independent read-only check.
#include "../geometry_base.hpp"
#include "../mesh_types.hpp"
#include "../mesh_validation.hpp"
#include "../predicates.hpp"
#include "../spatial_index.hpp"
#include <algorithm>
#include <array>
#include <deque>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pde2d::discretization::mesh_detail {
using Triangle = std::array<std::size_t, 3>;
inline Triangle ccw(Triangle t, const std::vector<Point2> &points) {
  int sign = predicates::orient(points[t[0]], points[t[1]], points[t[2]]);
  if (sign == 0)
    throw std::runtime_error("Operation would create a degenerate triangle");
  if (sign < 0)
    std::swap(t[1], t[2]);
  return t;
}
inline bool in_triangle(Point2 p, Point2 a, Point2 b, Point2 c) {
  return predicates::orient(a, b, p) >= 0 && predicates::orient(b, c, p) >= 0 &&
         predicates::orient(c, a, p) >= 0;
}

class Topology {
public:
  TriangleMesh &mesh;
  std::map<Edge, std::set<std::size_t>> owners;
  std::set<Edge> constrained;
  std::set<Edge> pending;
  std::map<Edge, std::size_t> generations;
  std::size_t generation = 0;
  struct SizeEntry {
    double length;
    Edge edge;
    std::size_t generation;
    bool operator<(const SizeEntry &b) const {
      if (length != b.length)
        return length < b.length;
      return edge > b.edge; // first lexicographic edge wins equal lengths
    }
  };
  std::priority_queue<SizeEntry> sizes;
  double target = 0;
  std::set<std::pair<double, double>> coordinates;
  // Work counters of the meshing call (MeshRunRecord::work), when set.
  // Counting never changes the mesh.
  MeshWork *work = nullptr;
  void queue_size(Edge e) {
    auto g = ++generation;
    generations[e] = g;
    if (target > 0) {
      double d = distance(mesh.nodes[e.first], mesh.nodes[e.second]);
      if (d > target * (1 + 1e-12))
        sizes.push({d, e, g});
    }
  }
  // Constant target h; zero or negative disables size queueing.
  void initialize_size(double h) {
    target = h;
    sizes = {};
    for (const auto &[e, own] : owners) {
      (void)own;
      queue_size(e);
    }
  }
  std::optional<Edge> longest() {
    while (!sizes.empty()) {
      auto t = sizes.top();
      auto g = generations.find(t.edge);
      if (!owners.contains(t.edge) || g == generations.end() ||
          g->second != t.generation) {
        sizes.pop();
        continue;
      }
      return t.edge;
    }
    return std::nullopt;
  }
  explicit Topology(TriangleMesh &m) : mesh(m) {
    for (auto p : m.nodes)
      coordinates.emplace(p.x, p.y);
    for (std::size_t i = 0; i < m.elements.size(); ++i)
      add(i);
    for (auto [a, b] : m.boundary_edges)
      constrained.insert(edge_key(a, b));
  }
  void add(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      auto &own = owners[key];
      own.insert(i);
      if (own.size() > 2)
        throw std::invalid_argument(
            "Nonmanifold mesh or incorrect boundary records");
      pending.insert(key);
      queue_size(key);
    }
  }
  void remove(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      auto it = owners.find(key);
      it->second.erase(i);
      if (it->second.empty()) {
        owners.erase(it);
        generations.erase(key);
      }
    }
  }
  std::size_t opposite(std::size_t t, Edge e) const {
    for (auto v : mesh.elements[t])
      if (v != e.first && v != e.second)
        return v;
    throw std::logic_error("Missing opposite vertex");
  }
  bool illegal(Edge e) const {
    auto it = owners.find(e);
    if (it == owners.end() || it->second.size() != 2 || constrained.contains(e))
      return false;
    auto i = *it->second.begin(), j = *std::next(it->second.begin()),
         c = opposite(i, e), d = opposite(j, e);
    auto a = e.first, b = e.second;
    if (predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[a]) *
            predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[b]) >=
        0)
      return false;
    if (predicates::orient(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c]) < 0)
      std::swap(a, b);
    return predicates::incircle(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c],
                                mesh.nodes[d]) > 0;
  }
  void legalize(bool global = true) {
    if (global)
      for (const auto &[e, own] : owners) {
        (void)own;
        pending.insert(e);
      }
    std::deque<Edge> queue(pending.begin(), pending.end());

    std::size_t flips = 0,
                limit = std::max<std::size_t>(
                    1000, mesh.elements.size() * mesh.elements.size() * 4);
    while (!queue.empty()) {
      auto edge = queue.front();
      queue.pop_front();
      pending.erase(edge);
      if (!illegal(edge))
        continue;
      if (++flips > limit)
        throw std::runtime_error("Delaunay legalization work budget exhausted");
      if (work)
        ++work->edge_flips;
      const auto own = owners.at(edge);
      auto i = *own.begin(), j = *std::next(own.begin()), c = opposite(i, edge),
           d = opposite(j, edge);
      auto first = ccw({c, d, edge.first}, mesh.nodes),
           second = ccw({d, c, edge.second}, mesh.nodes);
      remove(i);
      remove(j);
      mesh.elements[i] = first;
      mesh.elements[j] = second;
      // add() records size changes, including flip-created oversized edges.
      // Queue each affected edge once, allowing later re-enqueue after popping.
      std::set<Edge> affected;
      for (auto t : {first, second})
        for (int e = 0; e < 3; ++e)
          affected.insert(edge_key(t[e], t[(e + 1) % 3]));
      for (auto e : affected)
        if (!pending.contains(e))
          queue.push_back(e);
      add(i);
      add(j);
    }
  }
  void split(Edge edge, Point2 point) {
    auto it = owners.find(edge);
    if (it == owners.end())
      throw std::logic_error("Missing split edge");
    auto own = it->second;
    auto n = mesh.nodes.size();
    mesh.nodes.push_back(point);
    coordinates.emplace(point.x, point.y);
    for (auto i : own) {
      auto c = opposite(i, edge);
      auto a = ccw({edge.first, n, c}, mesh.nodes),
           b = ccw({n, edge.second, c}, mesh.nodes);
      remove(i);
      mesh.elements[i] = a;
      add(i);
      mesh.elements.push_back(b);
      add(mesh.elements.size() - 1);
    }
    if (constrained.erase(edge)) {
      constrained.insert(edge_key(edge.first, n));
      constrained.insert(edge_key(n, edge.second));
    }
  }
  void insert(Point2 point) {
    if (coordinates.contains({point.x, point.y}))
      return;
    for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
      auto t = mesh.elements[i];
      auto box = Box::segment(mesh.nodes[t[0]], mesh.nodes[t[1]]);
      auto p = mesh.nodes[t[2]];
      box = {std::min(box.xmin, p.x), std::max(box.xmax, p.x),
             std::min(box.ymin, p.y), std::max(box.ymax, p.y)};
      if (!box.overlaps(Box::segment(point, point)))
        continue;
      if (!in_triangle(point, mesh.nodes[t[0]], mesh.nodes[t[1]],
                       mesh.nodes[t[2]]))
        continue;
      for (int e = 0; e < 3; ++e)
        if (predicates::on_segment(mesh.nodes[t[e]], mesh.nodes[t[(e + 1) % 3]],
                                   point)) {
          split(edge_key(t[e], t[(e + 1) % 3]), point);
          return;
        }
      auto n = mesh.nodes.size();
      mesh.nodes.push_back(point);
      coordinates.emplace(point.x, point.y);
      remove(i);
      mesh.elements[i] = ccw({t[0], t[1], n}, mesh.nodes);
      add(i);
      mesh.elements.push_back(ccw({t[1], t[2], n}, mesh.nodes));
      add(mesh.elements.size() - 1);
      mesh.elements.push_back(ccw({t[2], t[0], n}, mesh.nodes));
      add(mesh.elements.size() - 1);
      return;
    }
    throw std::invalid_argument("Point lies outside the polygon");
  }
};

// Caller has already validated geometry. Read-only verification is independent
// of legalization scheduling and never copies the complete mesh.
inline bool cdt_verified(const TriangleMesh &mesh) {
  std::map<Edge, std::vector<std::size_t>> opposites;
  std::set<Edge> constrained;
  for (auto [a, b] : mesh.boundary_edges)
    constrained.insert(edge_key(a, b));
  for (auto t : mesh.elements)
    for (int e = 0; e < 3; ++e)
      opposites[edge_key(t[e], t[(e + 1) % 3])].push_back(t[(e + 2) % 3]);
  for (const auto &[edge, own] : opposites) {
    if (own.size() != 2 || constrained.contains(edge))
      continue;
    auto [a, b] = edge;
    auto c = own[0], d = own[1];
    if (predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[a]) *
            predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[b]) >=
        0)
      continue;
    if (predicates::orient(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c]) < 0)
      std::swap(a, b);
    if (predicates::incircle(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c],
                             mesh.nodes[d]) > 0)
      return false;
  }
  return true;
}
} // namespace pde2d::discretization::mesh_detail
