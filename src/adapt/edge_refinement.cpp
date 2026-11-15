#include "edge_refinement.hpp"

#include "pde2d/geometry/predicates.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace pde2d::adapt::detail {

using geometry::Point;
using mesh::no_index;

namespace {

using TriangleNodes = std::array<std::size_t, 3>;

// The edges of a mesh, numbered in increasing order of their (smaller,
// larger) node pair, so the numbering depends only on the mesh.
struct EdgeTable {
  std::vector<std::array<std::size_t, 2>> nodes; // a < b
  // For each triangle, the edge opposite each of its vertices: edges[t][i]
  // joins vertices (i + 1) % 3 and (i + 2) % 3.
  std::vector<std::array<std::size_t, 3>> of_triangle;
  // For each edge, its one or two triangles (no_index for the second on
  // the boundary).
  std::vector<std::array<std::size_t, 2>> triangles;
  // For each edge, its index in Mesh::boundary_edges, or no_index.
  std::vector<std::size_t> boundary;
};

EdgeTable edge_table(const mesh::Mesh &m) {
  struct HalfEdge {
    std::size_t a, b, triangle, local;
  };
  std::vector<HalfEdge> half;
  half.reserve(3 * m.triangles.size());
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    for (std::size_t i = 0; i < 3; ++i) {
      std::size_t u = m.triangles[t].nodes[(i + 1) % 3],
                  w = m.triangles[t].nodes[(i + 2) % 3];
      half.push_back({std::min(u, w), std::max(u, w), t, i});
    }
  std::sort(half.begin(), half.end(), [](const HalfEdge &x, const HalfEdge &y) {
    return std::tie(x.a, x.b, x.triangle, x.local) <
           std::tie(y.a, y.b, y.triangle, y.local);
  });
  EdgeTable table;
  table.of_triangle.resize(m.triangles.size());
  for (const auto &h : half) {
    if (table.nodes.empty() || table.nodes.back() != std::array{h.a, h.b}) {
      table.nodes.push_back({h.a, h.b});
      table.triangles.push_back({h.triangle, no_index});
    } else {
      table.triangles.back()[1] = h.triangle;
    }
    table.of_triangle[h.triangle][h.local] = table.nodes.size() - 1;
  }
  table.boundary.assign(table.nodes.size(), no_index);
  for (std::size_t k = 0; k < m.boundary_edges.size(); ++k) {
    auto [u, w] = m.boundary_edges[k].nodes;
    std::array key{std::min(u, w), std::max(u, w)};
    auto it = std::lower_bound(table.nodes.begin(), table.nodes.end(), key);
    // The mesh is valid, so every boundary edge is a triangle edge.
    table.boundary[static_cast<std::size_t>(it - table.nodes.begin())] = k;
  }
  return table;
}

std::string boundary_edge_text(const mesh::Mesh &m, std::size_t k) {
  const auto &e = m.boundary_edges[k];
  return "boundary edge " + std::to_string(k) + " (nodes " +
         std::to_string(e.nodes[0]) + " -> " + std::to_string(e.nodes[1]) +
         ") of region '" + m.region_names[e.region] + "'";
}

} // namespace

RefinementResult refine_marked_edges(const mesh::Mesh &mesh,
                                     std::span<const std::size_t> marked,
                                     const std::string &name,
                                     FullRefinement full) {
  const auto start = std::chrono::steady_clock::now();
  // The working mesh validates the input and knows where the boundary
  // split policy puts the point of each boundary edge.
  const mesh::WorkingMesh working = [&] {
    try {
      return mesh::WorkingMesh::from_mesh(mesh);
    } catch (const std::invalid_argument &e) {
      throw std::invalid_argument(name +
                                  ": the input mesh is not valid: " + e.what());
    }
  }();
  const std::size_t node_count = mesh.nodes.size();
  const std::size_t triangle_count = mesh.triangles.size();
  std::vector<bool> is_marked(triangle_count, false);
  for (auto t : marked) {
    if (t >= triangle_count)
      throw std::invalid_argument(
          name + ": marked triangle " + std::to_string(t) +
          " is out of range (the "
          "mesh has " +
          std::to_string(triangle_count) + " triangles)");
    is_marked[t] = true;
  }

  RefinementResult result;
  result.report.stage = name;
  const auto marked_count = static_cast<std::size_t>(
      std::count(is_marked.begin(), is_marked.end(), true));
  auto &metrics = result.report.metrics;
  if (marked_count == 0) {
    result.mesh = mesh;
    for (std::size_t v = 0; v < node_count; ++v)
      result.node_parents.push_back({v, v});
    for (std::size_t t = 0; t < triangle_count; ++t)
      result.triangle_parents.push_back(t);
    result.report.messages.push_back(
        "nothing was marked; the mesh is returned unchanged");
    for (const char *metric :
         {"marked", "refined_by_closure", "new_nodes", "new_triangles"})
      metrics[metric] = 0.0;
    result.report.time = std::chrono::steady_clock::now() - start;
    return result;
  }

  // 1. and 2. Mark edges and close the marking. A triangle is checked
  // whenever one of its edges becomes marked; the closed set does not
  // depend on the order (it is the smallest set containing the first
  // marks that is closed under the rule).
  const EdgeTable edges = edge_table(mesh);
  std::vector<bool> edge_marked(edges.nodes.size(), false);
  std::vector<std::size_t> to_check;
  auto mark_edge = [&](std::size_t e) {
    if (edge_marked[e])
      return;
    edge_marked[e] = true;
    for (auto t : edges.triangles[e])
      if (t != no_index)
        to_check.push_back(t);
  };
  for (std::size_t t = 0; t < triangle_count; ++t)
    if (is_marked[t])
      for (auto e : edges.of_triangle[t])
        mark_edge(e);
  while (!to_check.empty()) {
    std::size_t t = to_check.back();
    to_check.pop_back();
    // Some edge of t is marked (that is why it was queued), so its
    // refinement edge, edge 0, must be marked too.
    mark_edge(edges.of_triangle[t][0]);
  }

  // 3. A node on every marked edge, in edge order.
  mesh::Mesh &out = result.mesh;
  out.nodes = mesh.nodes;
  out.region_names = mesh.region_names;
  out.chains = mesh.chains;
  result.node_parents.reserve(node_count);
  for (std::size_t v = 0; v < node_count; ++v)
    result.node_parents.push_back({v, v});
  std::vector<std::size_t> edge_node(edges.nodes.size(), no_index);
  for (std::size_t e = 0; e < edges.nodes.size(); ++e) {
    if (!edge_marked[e])
      continue;
    auto [a, b] = edges.nodes[e];
    const std::size_t k = edges.boundary[e];
    // A boundary edge is split where the boundary split policy would split
    // it (the curve point for a linked edge, else the chord midpoint).
    const Point p = k != no_index
                        ? working.boundary_split_point(k)
                        : geometry::midpoint(mesh.nodes[a], mesh.nodes[b]);
    edge_node[e] = out.nodes.size();
    out.nodes.push_back(p);
    result.node_parents.push_back({a, b});
  }

  // 4. Divide the triangles, in index order.
  std::size_t refined_by_closure = 0;
  out.triangles.reserve(triangle_count);
  result.triangle_parents.reserve(triangle_count);
  std::vector<TriangleNodes> children;
  for (std::size_t t = 0; t < triangle_count; ++t) {
    const TriangleNodes &v = mesh.triangles[t].nodes;
    const auto &e = edges.of_triangle[t];
    const bool any =
        edge_marked[e[0]] || edge_marked[e[1]] || edge_marked[e[2]];
    children.clear();
    if (!any) {
      children.push_back(v);
    } else if (full == FullRefinement::red && edge_marked[e[0]] &&
               edge_marked[e[1]] && edge_marked[e[2]]) {
      // Red: m_i is the midpoint of the edge opposite v_i. Each child
      // lists the images of v0, v1, v2 in that order (see
      // red_green_blue_refinement.hpp).
      const std::size_t m0 = edge_node[e[0]], m1 = edge_node[e[1]],
                        m2 = edge_node[e[2]];
      children.push_back({v[0], m2, m1});
      children.push_back({m2, v[1], m0});
      children.push_back({m1, m0, v[2]});
      children.push_back({m0, m1, m2});
    } else {
      // Bisection while the refinement edge (opposite vertex 0) is
      // marked. Only edges of t itself can be marked: the edges made
      // inside t are new.
      auto node_on = [&](std::size_t a, std::size_t b) {
        for (auto edge : e) {
          auto [x, y] = edges.nodes[edge];
          if ((x == a && y == b) || (x == b && y == a))
            return edge_node[edge];
        }
        return no_index;
      };
      auto bisect = [&](auto &self, TriangleNodes tri) -> void {
        const std::size_t m = node_on(tri[1], tri[2]);
        if (m == no_index) {
          children.push_back(tri);
          return;
        }
        // (v0, v1, v2) with refinement edge v1-v2 -> (m, v0, v1) and
        // (m, v2, v0); both counterclockwise, m first.
        self(self, {m, tri[0], tri[1]});
        self(self, {m, tri[2], tri[0]});
      };
      bisect(bisect, v);
    }
    if (any && !is_marked[t])
      ++refined_by_closure;
    // Every child must be counterclockwise. Midpoints of straight edges
    // always give that; a curve point can be too far from its chord.
    for (const auto &c : children) {
      if (geometry::orient(out.nodes[c[0]], out.nodes[c[1]], out.nodes[c[2]]) >
          0)
        continue;
      for (auto edge : e)
        if (edges.boundary[edge] != no_index && edge_marked[edge] &&
            std::find(c.begin(), c.end(), edge_node[edge]) != c.end())
          throw std::runtime_error(
              name + ": splitting " +
              boundary_edge_text(mesh, edges.boundary[edge]) +
              " at its curve point would invert a child of triangle " +
              std::to_string(t) +
              ": the boundary split policy would have to re-triangulate "
              "there, which refinement does not do (the boundary chords are "
              "too coarse for the curve; refine or remesh this part first)");
      throw std::runtime_error(name + ": a child of triangle " +
                               std::to_string(t) +
                               " is degenerate (the triangle is too thin to "
                               "split in floating point)");
    }
    for (const auto &c : children) {
      out.triangles.push_back({c});
      result.triangle_parents.push_back(t);
    }
  }

  // 5. Boundary edges: a split edge is replaced in place by its halves.
  out.boundary_edges.reserve(mesh.boundary_edges.size());
  for (std::size_t k = 0; k < mesh.boundary_edges.size(); ++k) {
    const mesh::BoundaryEdge &be = mesh.boundary_edges[k];
    auto [a, b] = be.nodes;
    auto key = std::array{std::min(a, b), std::max(a, b)};
    auto it = std::lower_bound(edges.nodes.begin(), edges.nodes.end(), key);
    const std::size_t e = static_cast<std::size_t>(it - edges.nodes.begin());
    if (!edge_marked[e]) {
      out.boundary_edges.push_back(be);
      continue;
    }
    const std::size_t m = edge_node[e];
    mesh::BoundaryEdge first{{a, m}, be.region, be.link};
    mesh::BoundaryEdge second{{m, b}, be.region, be.link};
    if (be.link) {
      // The same arc-length middle as the point (and as the boundary split
      // policy's halves).
      const double s = 0.5 * (be.link->s0 + be.link->s1);
      if (!(be.link->s0 < s && s < be.link->s1))
        throw std::runtime_error(name + ": " + boundary_edge_text(mesh, k) +
                                 " is too short in arc length to split");
      first.link->s1 = s;
      second.link->s0 = s;
    }
    out.boundary_edges.push_back(first);
    out.boundary_edges.push_back(second);
  }

  // The output must be valid. With straight edges it is by construction
  // (every child lies in its parent); a curve point bulging out of the
  // mesh can overlap another part of it, which only this check sees.
  try {
    mesh::validate(out);
  } catch (const std::invalid_argument &e) {
    throw std::runtime_error(
        name +
        ": a curve point of a split boundary edge cannot be placed by "
        "a plain split: the refined mesh would not be valid (" +
        std::string(e.what()) +
        "); the boundary split policy would have to re-triangulate there, "
        "which refinement does not do");
  }

  metrics["marked"] = static_cast<double>(marked_count);
  metrics["refined_by_closure"] = static_cast<double>(refined_by_closure);
  metrics["new_nodes"] = static_cast<double>(out.nodes.size() - node_count);
  metrics["new_triangles"] =
      static_cast<double>(out.triangles.size() - triangle_count);
  result.report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::adapt::detail
