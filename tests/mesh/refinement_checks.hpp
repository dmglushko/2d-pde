#pragma once

// Independent checks of a refinement result (Remesher::refine), shared by
// the refinement tests. They use plain long double arithmetic and std::map
// / std::set, not the library's predicates or edge tables.

#include "support/check.hpp"
#include "support/mesh_checks.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/red_green_blue_refinement.hpp"
#include "pde2d/mesh/remesher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <span>
#include <utility>
#include <vector>

namespace pde2d::test {

using pde2d::mesh::Parents;
using pde2d::mesh::RefinementResult;
using Marked = std::vector<std::size_t>;

// The three bisection refiners.
inline std::vector<std::shared_ptr<const mesh::Refiner>> bisection_refiners() {
  return {std::make_shared<mesh::LongestEdgeBisection>(),
          std::make_shared<mesh::NewestVertexBisection>(),
          std::make_shared<mesh::RedGreenBlueRefinement>()};
}

// How many children a marked triangle has at least: two after one
// bisection, four when all its edges are halved (red-green-blue).
inline std::size_t marked_children(const mesh::Refiner &refiner) {
  return refiner.name() == "red_green_blue" ? 4 : 2;
}

// Refines with a Remesher that has only this refiner.
inline RefinementResult
refine_with(const std::shared_ptr<const mesh::Refiner> &refiner,
            const Mesh &mesh, std::span<const std::size_t> marked) {
  return mesh::Remesher(refiner).refine(mesh, marked);
}

// The refiner's report (after the pipeline report).
inline const mesh::StageReport &refiner_report(const RefinementResult &r) {
  return r.reports.at(1);
}

// A mesh of the geometry (chord-length discretizer and CDT, by default no
// refiner), with its refinement edges set to the longest edges.
// With a refiner (for example Ruppert) the mesh also has interior nodes.
inline Mesh
initial_mesh(const geometry::CurveGeometry &g, double h,
             std::shared_ptr<const mesh::Refiner> refiner = nullptr) {
  mesh::Mesher mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                      std::make_shared<mesh::Cdt>(), std::move(refiner));
  Mesh m = mesher.run(g, {.max_edge_length = {.absolute = h}}).mesh;
  mesh::set_longest_edges_as_refinement_edges(m);
  return m;
}

inline std::array<Point, 3> corners(const Mesh &m, std::size_t t) {
  const auto &v = m.triangles[t].nodes;
  return {m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]]};
}

// p in the closed counterclockwise triangle, with a small tolerance.
inline bool in_triangle(Point p, const std::array<Point, 3> &c) {
  for (int i = 0; i < 3; ++i) {
    Point a = c[i], b = c[(i + 1) % 3];
    if (cross_ld(a, b, p) < -cross_tolerance(a, b, p))
      return false;
  }
  return true;
}

inline std::pair<std::size_t, std::size_t> undirected(std::size_t a,
                                                      std::size_t b) {
  return {std::min(a, b), std::max(a, b)};
}

// Every input edge (undirected) and the triangles that use it.
inline std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>>
edges_of(const Mesh &m) {
  std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> e;
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    for (int i = 0; i < 3; ++i)
      e[undirected(m.triangles[t].nodes[i], m.triangles[t].nodes[(i + 1) % 3])]
          .push_back(t);
  return e;
}

// What the checks may expect of the geometry. On a straight domain (or a
// mesh without curve links) every new node is the exact midpoint of its
// edge, every child lies in its parent and area is preserved. Near a
// curved boundary none of that holds for the triangles on it, so a test
// of a curved domain sets `straight` to false and checks the curve points
// itself.
struct RefinementExpectations {
  bool straight = true;
  // Children of each marked triangle, at least (marked_children).
  std::size_t marked_children = 2;
};

// The checks every refinement result must pass, given its input and the
// marked triangles:
//  - valid conforming mesh (check_valid_mesh: orientation, conformity,
//    boundary edges = edges used once, no overlaps);
//  - input nodes keep their index and place; every new node lies on an
//    input edge (its parents), at the midpoint when `straight`;
//  - every child names a parent; its corners are the parent's corners or
//    nodes on the parent's edges; with `straight` the children lie in the
//    parent and their areas add up to it;
//  - every marked triangle has at least expect.marked_children children;
//    an unrefined triangle is copied with its vertex order;
//  - boundary edges: an unsplit edge is kept with its region and link, a
//    split one becomes two halves through the new node with its region;
//    links stay valid;
//  - the report's counts and the measured properties.
inline void check_refinement(const Mesh &in, const RefinementResult &r,
                             std::span<const std::size_t> marked,
                             RefinementExpectations expect = {}) {
  const Mesh &out = r.mesh;
  check_valid_mesh(out);
  check(r.parents.has_value(), "the result has parents");
  if (!r.parents)
    return;
  const Parents &parents = *r.parents;
  const std::size_t n = in.nodes.size();
  check(out.region_names == in.region_names, "region names kept");
  check(out.chains.size() == in.chains.size(), "chains kept");
  check(parents.nodes.size() == out.nodes.size(), "a parent per node");
  check(parents.triangles.size() == out.triangles.size(),
        "a parent per triangle");
  check(out.nodes.size() >= n, "no node removed");

  const auto input_edges = edges_of(in);
  for (std::size_t v = 0; v < out.nodes.size(); ++v) {
    auto [a, b] = parents.nodes[v];
    if (v < n) {
      check(a == v && b == v, "an input node is its own parent");
      check(out.nodes[v] == in.nodes[v], "an input node does not move");
      continue;
    }
    check(a < b && b < n, "a new node has two input nodes as parents");
    check(input_edges.count({a, b}) == 1, "a new node's parents form an edge");
    if (expect.straight) {
      Point m{0.5 * (in.nodes[a].x + in.nodes[b].x),
              0.5 * (in.nodes[a].y + in.nodes[b].y)};
      check(out.nodes[v] == m, "a new node is the midpoint of its edge");
    }
  }
  std::set<std::pair<std::size_t, std::size_t>> split_edges;
  for (std::size_t v = n; v < out.nodes.size(); ++v)
    check(split_edges.insert({parents.nodes[v][0], parents.nodes[v][1]}).second,
          "one new node per split edge");

  // Children grouped by parent.
  std::map<std::size_t, std::vector<std::size_t>> children;
  for (std::size_t t = 0; t < out.triangles.size(); ++t) {
    std::size_t p = parents.triangles[t];
    check(p < in.triangles.size(), "parent index in range");
    children[p].push_back(t);
  }
  check(children.size() == in.triangles.size(), "every input triangle has "
                                                "children (or itself)");
  std::set<std::size_t> distinct(marked.begin(), marked.end());
  std::size_t refined_unmarked = 0;
  for (const auto &[p, list] : children) {
    const auto &pv = in.triangles[p].nodes;
    if (list.size() == 1) {
      check(out.triangles[list[0]].nodes == pv,
            "an unrefined triangle is copied with its vertex order");
      continue;
    }
    refined_unmarked += distinct.count(p) == 0 ? 1 : 0;
    check(list.size() <= 4, "at most four children");
    const auto pc = corners(in, p);
    long double area = 0;
    for (auto t : list) {
      for (auto v : out.triangles[t].nodes) {
        if (v < n) {
          check(std::find(pv.begin(), pv.end(), v) != pv.end(),
                "a child's input corner is a corner of its parent");
        } else {
          auto [a, b] = parents.nodes[v];
          check(std::find(pv.begin(), pv.end(), a) != pv.end() &&
                    std::find(pv.begin(), pv.end(), b) != pv.end(),
                "a child's new corner lies on an edge of its parent");
        }
        if (expect.straight)
          check(in_triangle(out.nodes[v], pc), "a child lies in its parent");
      }
      auto c = corners(out, t);
      area += triangle_area(c[0], c[1], c[2]);
    }
    if (expect.straight)
      near(static_cast<double>(area), triangle_area(pc[0], pc[1], pc[2]),
           1e-12 * triangle_area(pc[0], pc[1], pc[2]),
           "children's areas add up to their parent's");
  }
  for (auto t : distinct)
    check(children[t].size() >= expect.marked_children,
          "a marked triangle has at least " +
              std::to_string(expect.marked_children) + " children");
  if (expect.straight)
    near(mesh_area(out), mesh_area(in), 1e-12 * mesh_area(in),
         "area preserved");

  // Boundary edges.
  std::map<std::pair<std::size_t, std::size_t>, const mesh::BoundaryEdge *>
      input_boundary;
  for (const auto &e : in.boundary_edges)
    input_boundary[{e.nodes[0], e.nodes[1]}] = &e;
  std::size_t split_boundary = 0;
  for (const auto &e : out.boundary_edges) {
    auto [u, w] = e.nodes;
    if (e.link)
      check(mesh::is_valid_link(*e.link, out.chains), "links stay valid");
    if (u < n && w < n) {
      auto it = input_boundary.find({u, w});
      check(it != input_boundary.end(), "an old boundary edge was a boundary "
                                        "edge");
      check(it->second->region == e.region, "region kept");
      check(it->second->link.has_value() == e.link.has_value(), "link kept");
      check(split_edges.count(undirected(u, w)) == 0,
            "an unsplit boundary edge was not split");
      continue;
    }
    check((u < n) != (w < n), "a half has one old and one new node");
    std::size_t m = u < n ? w : u;
    auto [a, b] = parents.nodes[m];
    auto it = input_boundary.find({a, b});
    if (it == input_boundary.end())
      it = input_boundary.find({b, a});
    check(it != input_boundary.end(), "a half lies on an input boundary "
                                      "edge");
    const auto &parent = *it->second;
    check(parent.region == e.region, "a half keeps the region");
    check(parent.link.has_value() == e.link.has_value(),
          "a half of a linked edge is linked");
    // The half goes the same way as its parent: (a', m) or (m, b').
    check((u == parent.nodes[0] && w == m) || (u == m && w == parent.nodes[1]),
          "a half keeps the direction of its edge");
    if (parent.link) {
      const double s = 0.5 * (parent.link->s0 + parent.link->s1);
      if (u == m)
        check(e.link->s0 == s && e.link->s1 == parent.link->s1,
              "second half of the link");
      else
        check(e.link->s0 == parent.link->s0 && e.link->s1 == s,
              "first half of the link");
    }
    ++split_boundary;
  }
  check(out.boundary_edges.size() ==
            in.boundary_edges.size() + split_boundary / 2,
        "each split boundary edge gives two halves");

  const auto &metrics = refiner_report(r).metrics;
  near(metrics.at("marked"), static_cast<double>(distinct.size()), 0,
       "reported marked count");
  near(metrics.at("refined_by_closure"), static_cast<double>(refined_unmarked),
       0, "reported closure count");
  check(r.properties.nodes == out.nodes.size() &&
            r.properties.triangles == out.triangles.size(),
        "measured counts");
}

// Same refined mesh, links and parents.
inline bool same_result(const RefinementResult &a, const RefinementResult &b) {
  if (!same_mesh(a.mesh, b.mesh) ||
      a.parents.has_value() != b.parents.has_value())
    return false;
  if (a.parents && (a.parents->triangles != b.parents->triangles ||
                    a.parents->nodes != b.parents->nodes))
    return false;
  for (std::size_t k = 0; k < a.mesh.boundary_edges.size(); ++k) {
    const auto &x = a.mesh.boundary_edges[k].link,
               &y = b.mesh.boundary_edges[k].link;
    if (x.has_value() != y.has_value())
      return false;
    if (x && (x->chain != y->chain || x->s0 != y->s0 || x->s1 != y->s1))
      return false;
  }
  return true;
}

// The shapes present in a mesh, as sorted angle triples (degrees), merged
// when they agree to within `tolerance`: the number of similarity classes
// (mirror images count as one).
inline std::vector<std::array<double, 3>>
shape_classes(const Mesh &m, std::vector<std::array<double, 3>> known = {},
              double tolerance = 1e-6) {
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    auto c = corners(m, t);
    std::array<double, 3> angle{};
    for (int i = 0; i < 3; ++i) {
      Point a = c[i], b = c[(i + 1) % 3], d = c[(i + 2) % 3];
      double ux = b.x - a.x, uy = b.y - a.y, vx = d.x - a.x, vy = d.y - a.y;
      angle[i] = std::atan2(std::abs(ux * vy - uy * vx), ux * vx + uy * vy) *
                 180 / std::numbers::pi;
    }
    std::sort(angle.begin(), angle.end());
    bool found = false;
    for (const auto &k : known)
      if (std::abs(k[0] - angle[0]) <= tolerance &&
          std::abs(k[1] - angle[1]) <= tolerance &&
          std::abs(k[2] - angle[2]) <= tolerance) {
        found = true;
        break;
      }
    if (!found)
      known.push_back(angle);
  }
  return known;
}

} // namespace pde2d::test
