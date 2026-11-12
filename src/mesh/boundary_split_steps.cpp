#include "boundary_split_steps.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::mesh::detail {

using geometry::orient;
using geometry::Point;

namespace {

std::size_t next(std::size_t i) { return (i + 1) % 3; }
std::size_t prev(std::size_t i) { return (i + 2) % 3; }

std::string edge_text(std::size_t a, std::size_t b) {
  return "(" + std::to_string(a) + ", " + std::to_string(b) + ")";
}

// Boundary edge `e` split into halves at node q: e keeps a -> q, the
// returned edge is q -> b. Links are halved in arc length.
WorkingBoundaryEdge split_edge_record(WorkingBoundaryEdge &e, std::size_t q) {
  WorkingBoundaryEdge second{{q, e.nodes[1]}, e.region, std::nullopt};
  if (e.link) {
    double sm = 0.5 * (e.link->s0 + e.link->s1);
    second.link = CurveLink{e.link->chain, sm, e.link->s1};
    e.link->s1 = sm;
  }
  e.nodes[1] = q;
  return second;
}

} // namespace

std::array<std::size_t, 2> BoundarySplitSteps::split_directly(WorkingMesh &m,
                                                              std::size_t k) {
  auto [t, edge] = m.boundary_edge_location(k);
  if (t == no_index)
    throw std::logic_error("Boundary edge is not in the triangulation");
  auto i = static_cast<std::size_t>(edge);
  // t = (c, a, b) with the boundary edge a -> b.
  std::size_t c = m.triangles_[t][i], a = m.triangles_[t][next(i)],
              b = m.triangles_[t][prev(i)];
  Point p = m.boundary_split_point(k);
  if (orient(m.nodes_[c], m.nodes_[a], p) <= 0 ||
      orient(m.nodes_[c], p, m.nodes_[b]) <= 0)
    throw std::runtime_error(
        "Splitting boundary edge " + edge_text(a, b) + " of region '" +
        m.region_names_[m.boundary_edges_[k].region] +
        "' at its curve point would invert a triangle; the boundary chords "
        "are too coarse for the curve here (use a smaller curve deviation "
        "or edge length)");
  std::size_t n_bc = m.neighbours_[t][next(i)],
              n_ca = m.neighbours_[t][prev(i)];
  std::size_t q = m.nodes_.size(), n1 = m.triangles_.size();
  m.nodes_.push_back(p);
  m.node_triangle_.push_back(t);
  m.triangles_[t] = {c, a, q};
  m.neighbours_[t] = {no_index, n1, n_ca};
  m.triangles_.push_back({c, q, b});
  m.neighbours_.push_back({no_index, n_bc, t});
  m.replace_neighbour(n_bc, t, n1);
  m.set_node_triangles(t);
  m.set_node_triangles(n1);

  auto second = split_edge_record(m.boundary_edges_[k], q);
  m.boundary_edges_.push_back(second);
  m.unsplittable_.push_back(false);
  m.outgoing_boundary_.push_back(m.boundary_edges_.size() - 1);
  return {t, n1};
}

std::array<std::size_t, 2> BoundarySplitSteps::add_cap(WorkingMesh &m,
                                                       std::size_t k) {
  auto [t, edge] = m.boundary_edge_location(k);
  if (t == no_index)
    throw std::logic_error("Boundary edge is not in the triangulation");
  auto [a, b] = m.boundary_edges_[k].nodes;
  Point p = m.boundary_split_point(k);
  if (orient(m.nodes_[a], m.nodes_[b], p) >= 0)
    throw std::invalid_argument(
        "add_cap: the split point is not outside the boundary edge");
  std::size_t q = m.nodes_.size(), cap = m.triangles_.size();
  m.nodes_.push_back(p);
  m.node_triangle_.push_back(cap);
  // cap = (a, q, b): edge 0 (q -> b) and edge 2 (a -> q) are boundary, edge
  // 1 (b -> a) faces t.
  m.triangles_.push_back({a, q, b});
  m.neighbours_.push_back({no_index, t, no_index});
  m.neighbours_[t][static_cast<std::size_t>(edge)] = cap;
  auto second = split_edge_record(m.boundary_edges_[k], q);
  m.boundary_edges_.push_back(second);
  m.unsplittable_.push_back(false);
  m.outgoing_boundary_.push_back(m.boundary_edges_.size() - 1);
  return {t, cap};
}

std::array<std::size_t, 2>
BoundarySplitSteps::cut_off(WorkingMesh &m, std::size_t k, std::size_t q,
                            std::size_t *deleted_nodes,
                            std::size_t *deleted_triangles) {
  auto [a, b] = m.boundary_edges_.at(k).nodes;
  if (q >= m.nodes_.size() || m.is_node_deleted(q) || m.is_boundary_node(q))
    throw std::invalid_argument("cut_off: q must be a live interior node");
  if (!(m.nodes_[q] == m.boundary_split_point(k)))
    throw std::invalid_argument("cut_off: q is not at the edge's split point");
  // Kept triangles: on the left of a -> q and q -> b. The region to cut
  // lies across those edges.
  auto [keep_a, edge_a] = m.find_edge(a, q);
  auto [keep_b, edge_b] = m.find_edge(q, b);
  if (keep_a == no_index || keep_b == no_index ||
      m.neighbour(keep_a, edge_a) == no_index ||
      m.neighbour(keep_b, edge_b) == no_index)
    throw std::invalid_argument("cut_off: a-q and q-b must be interior edges");
  // The region: triangles reachable from the one on a -> b without
  // crossing a-q or q-b.
  auto [start, start_edge] = m.boundary_edge_location(k);
  (void)start_edge;
  auto is_cut_edge = [&](std::size_t u, std::size_t v) {
    return (u == q && v == a) || (u == b && v == q);
  };
  std::vector<std::size_t> region{start};
  std::vector<bool> in_region(m.triangles_.size(), false);
  in_region[start] = true;
  for (std::size_t i = 0; i < region.size(); ++i) {
    std::size_t t = region[i];
    for (std::size_t j = 0; j < 3; ++j) {
      std::size_t u = m.triangles_[t][next(j)], v = m.triangles_[t][prev(j)];
      if (is_cut_edge(u, v))
        continue;
      std::size_t n = m.neighbours_[t][j];
      if (n == no_index) {
        if (!(u == a && v == b))
          throw std::invalid_argument(
              "cut_off: the region contains another boundary "
              "edge");
        continue;
      }
      if (!in_region[n]) {
        in_region[n] = true;
        region.push_back(n);
      }
    }
  }
  if (in_region[keep_a] || in_region[keep_b])
    throw std::invalid_argument("cut_off: a-q-b does not separate a region");
  // Nodes inside the region: every vertex other than a, b, q. All of their
  // triangles must be in the region.
  std::vector<std::size_t> inner;
  for (auto t : region)
    for (auto v : m.triangles_[t])
      if (v != a && v != b && v != q &&
          std::find(inner.begin(), inner.end(), v) == inner.end()) {
        if (m.is_boundary_node(v))
          throw std::invalid_argument(
              "cut_off: the region contains a boundary node");
        for (auto s : m.triangles_around(v))
          if (!in_region[s])
            throw std::invalid_argument(
                "cut_off: a node of the region is used outside "
                "it");
        inner.push_back(v);
      }

  // Change the mesh.
  m.neighbours_[keep_a][static_cast<std::size_t>(edge_a)] = no_index;
  m.neighbours_[keep_b][static_cast<std::size_t>(edge_b)] = no_index;
  for (auto t : region) {
    m.triangles_[t] = {no_index, no_index, no_index};
    m.neighbours_[t] = {no_index, no_index, no_index};
  }
  m.deleted_triangles_ += region.size();
  for (auto v : inner)
    m.node_triangle_[v] = no_index;
  m.node_triangle_[a] = keep_a;
  m.node_triangle_[q] = keep_a;
  m.node_triangle_[b] = keep_b;
  auto second = split_edge_record(m.boundary_edges_[k], q);
  m.boundary_edges_.push_back(second);
  m.unsplittable_.push_back(false);
  m.outgoing_boundary_[q] = m.boundary_edges_.size() - 1;
  if (deleted_nodes)
    *deleted_nodes = inner.size();
  if (deleted_triangles)
    *deleted_triangles = region.size();
  return {keep_a, keep_b};
}

} // namespace pde2d::mesh::detail
