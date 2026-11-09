#pragma once
// Private (mesh_detail) part of triangulate.hpp: what the generic refinement
// loop plugs together.
//
// * Size criteria (RefinementCriterion, public in size_criterion.hpp): the
//   constant max_edge_length, a SizeField, a user criterion and marked edges
//   all reduce to one SizeCriterion that accepts an edge or gives its
//   splitting priority. size_criterion() turns RefinementOptions into one.
// * Boundary policies: one struct per BoundaryTreatment (FixedBoundary,
//   SplitPolygonBoundary, CurveBoundary) holding everything that treatment
//   decides: required context, preparation, the boundary conflict check and
//   how a boundary edge is split. with_boundary_policy() is the only
//   dispatch.
#include "../mesh_types.hpp"
#include "../size_criterion.hpp"
#include "cdt_topology.hpp"
#include "curved_boundary.hpp"
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::discretization::mesh_detail {
// The criterion of a refinement request. Without a size field this is
// exactly the constant-target criterion (ranked by length); a field ranks
// edges by length relative to the local target. A user criterion
// (RefinementOptions::criterion) replaces both; validateMeshOptionValues
// rejects setting it together with either.
inline RefinementCriterion size_criterion(const RefinementOptions &o) {
  if (o.criterion)
    return *o.criterion;
  if (!o.size_field)
    return {constant_size_criterion(o.max_edge_length),
            "Fixed boundary edge exceeds requested maximum triangle-edge "
            "length"};
  return {size_field_criterion(o.size_field, o.max_edge_length),
          "Fixed boundary edge exceeds the local size target"};
}

// Marked refinement: each marked triangle selects one edge to bisect, keyed
// by endpoint coordinates because curved-boundary rebuilds renumber
// vertices. Interior edges (tier 2) go before boundary edges (tier 1) so a
// curved rebuild cannot discard interior selections; any size target
// (tier 0) is enforced afterwards.
struct CoordinateEdge {
  std::pair<double, double> first, second;
  auto operator<=>(const CoordinateEdge &) const = default;
};
inline CoordinateEdge coordinate_edge(Point2 a, Point2 b) {
  std::pair<double, double> p{a.x, a.y}, q{b.x, b.y};
  if (q < p)
    std::swap(p, q);
  return {p, q};
}
struct MarkedState {
  std::map<CoordinateEdge, int> pending; // edge -> tier
  std::size_t marked_elements = 0, marked_edges = 0, bisected = 0,
              unrefinable = 0;
};
// Longest edge of each marked triangle that the boundary treatment may split
// (ties: first local edge). Without a splittable edge the triangle is
// counted as unrefinable.
inline std::shared_ptr<MarkedState>
select_marked_edges(const TriangleMesh &mesh,
                    const std::vector<std::size_t> &marked,
                    bool boundary_splittable) {
  auto state = std::make_shared<MarkedState>();
  std::set<Edge> boundary;
  for (auto [a, b] : mesh.boundary_edges)
    boundary.insert(edge_key(a, b));
  std::set<std::size_t> unique;
  for (auto i : marked) {
    if (i >= mesh.elements.size())
      throw std::invalid_argument("Marked element index is out of range");
    unique.insert(i);
  }
  state->marked_elements = unique.size();
  for (auto i : unique) {
    const auto &t = mesh.elements[i];
    std::optional<Edge> best;
    double best_length = 0;
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      if (!boundary_splittable && boundary.contains(key))
        continue;
      double d = distance(mesh.nodes[key.first], mesh.nodes[key.second]);
      if (!best || d > best_length) {
        best = key;
        best_length = d;
      }
    }
    if (!best) {
      ++state->unrefinable;
      continue;
    }
    state->pending[coordinate_edge(mesh.nodes[best->first],
                                   mesh.nodes[best->second])] =
        boundary.contains(*best) ? 1 : 2;
  }
  state->marked_edges = state->pending.size();
  return state;
}
inline RefinementCriterion marked_criterion(std::shared_ptr<MarkedState> state,
                                            RefinementCriterion size) {
  RefinementCriterion c;
  c.fixed_conflict = std::move(size.fixed_conflict);
  c.descriptor = std::move(size.descriptor);
  c.size = [state, inner = std::move(size.size)](
               Point2 a, Point2 b) -> std::optional<SizePriority> {
    auto it = state->pending.find(coordinate_edge(a, b));
    if (it != state->pending.end()) {
      const double d = distance(a, b);
      return SizePriority{it->second, d, d};
    }
    if (inner)
      return inner(a, b);
    return std::nullopt;
  };
  c.split_hook = [state, inner = std::move(size.split_hook)](Point2 a,
                                                             Point2 b) {
    if (state->pending.erase(coordinate_edge(a, b)))
      ++state->bisected;
    if (inner)
      inner(a, b);
  };
  return c;
}

// Mutable loop state that boundary policies may replace (curve rebuilds
// create a new topology).
struct RefinementState {
  MeshResult &result;
  std::unique_ptr<Topology> topology;
  const RefinementCriterion &criterion;
  bool delaunay;
};

// Whether any boundary edge fails the criterion.
inline bool boundary_fails(const MeshResult &r, const RefinementCriterion &c) {
  for (auto [a, b] : r.mesh.boundary_edges)
    if (c.size(r.mesh.nodes[a], r.mesh.nodes[b]))
      return true;
  return false;
}

// BoundaryTreatment::Fixed: keep every boundary vertex and edge. A boundary
// edge failing the criterion makes the request impossible.
struct FixedBoundary {
  static void validate(const MeshResult &) {}
  static void prepare(MeshResult &, const RefinementCriterion &) {}
  static std::optional<std::string> conflict(const MeshResult &r,
                                             const RefinementCriterion &c) {
    if (boundary_fails(r, c))
      return c.fixed_conflict;
    return std::nullopt;
  }
  static void split_boundary(RefinementState &, Edge, std::size_t, Point2) {
    throw std::logic_error("Fixed boundary edges are never split");
  }
};

// BoundaryTreatment::SplitPolygonEdges: midpoints on the straight chords;
// source intervals are still bisected by arc length.
struct SplitPolygonBoundary {
  static void validate(const MeshResult &) {}
  static void prepare(MeshResult &, const RefinementCriterion &) {}
  static std::optional<std::string> conflict(const MeshResult &,
                                             const RefinementCriterion &) {
    return std::nullopt;
  }
  static void split_boundary(RefinementState &s, Edge edge, std::size_t index,
                             Point2 mid) {
    auto n = s.result.mesh.nodes.size();
    s.topology->split(edge, mid);
    split_boundary_record(s.result, index, n);
    if (s.delaunay)
      s.topology->legalize(false);
  }
};

// BoundaryTreatment::RefineOnCurves: new boundary vertices on the analytical
// source curves; every boundary split rebuilds the triangulation.
struct CurveBoundary {
  static void validate(const MeshResult &r) {
    if (!r.boundary_context)
      throw std::invalid_argument(
          "Curve refinement requires original boundary context");
  }
  // Chord-split vertices (from SplitPolygonEdges) move onto the curves first.
  static void prepare(MeshResult &r, const RefinementCriterion &c) {
    if (!c.active())
      return;
    bool off_curve = false;
    for (std::size_t i = 0; i < r.mesh.boundary_edges.size(); ++i) {
      auto s = r.boundary_context->edge_sources[i].front();
      auto p = curvePoint(r.boundary_context->curves[s.curve].segment, s.begin);
      if (distance(p, r.mesh.nodes[r.mesh.boundary_edges[i].first]) > 0)
        off_curve = true;
    }
    if (off_curve)
      rebuild_curved(r);
  }
  static std::optional<std::string> conflict(const MeshResult &,
                                             const RefinementCriterion &) {
    return std::nullopt;
  }
  static void split_boundary(RefinementState &s, Edge, std::size_t index,
                             Point2 mid) {
    // A temporary vertex supplies connectivity; rebuild samples its actual
    // curve parameter and does not reuse its midpoint coordinates.
    auto n = s.result.mesh.nodes.size();
    s.result.mesh.nodes.push_back(mid);
    split_boundary_record(s.result, index, n);
    rebuild_curved(s.result);
    s.topology = std::make_unique<Topology>(s.result.mesh);
    s.topology->work = &s.result.record.work;
    s.topology->initialize_size(s.criterion.size);
  }
};

template <class F>
decltype(auto) with_boundary_policy(BoundaryTreatment treatment, F &&f) {
  switch (treatment) {
  case BoundaryTreatment::Fixed:
    return f(FixedBoundary{});
  case BoundaryTreatment::SplitPolygonEdges:
    return f(SplitPolygonBoundary{});
  case BoundaryTreatment::RefineOnCurves:
    return f(CurveBoundary{});
  }
  throw std::invalid_argument("Unknown boundary treatment");
}
} // namespace pde2d::discretization::mesh_detail
