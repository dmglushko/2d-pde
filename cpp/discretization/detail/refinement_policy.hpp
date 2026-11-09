#pragma once
// Private (mesh_detail) part of triangulate.hpp: the boundary policies of
// the refinement loop (refinement_loop.hpp). One struct per
// BoundaryTreatment (FixedBoundary, SplitPolygonBoundary, CurveBoundary)
// holds everything that treatment decides: required context, preparation,
// the boundary conflict check and how a boundary edge is split.
// with_boundary_policy() is the only dispatch.
#include "../mesh_types.hpp"
#include "cdt_topology.hpp"
#include "curved_boundary.hpp"
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace pde2d::discretization::mesh_detail {
// Mutable loop state that boundary policies may replace (curve rebuilds
// create a new topology).
struct RefinementState {
  MeshResult &result;
  std::unique_ptr<Topology> topology;
  double target; // RefinementOptions::max_edge_length
  bool delaunay;
};

// BoundaryTreatment::Fixed: keep every boundary vertex and edge. A boundary
// edge longer than the target makes the request impossible.
struct FixedBoundary {
  static void validate(const MeshResult &) {}
  static void prepare(MeshResult &, double) {}
  static std::optional<std::string> conflict(const MeshResult &r, double h) {
    for (auto [a, b] : r.mesh.boundary_edges)
      if (distance(r.mesh.nodes[a], r.mesh.nodes[b]) > h * (1 + 1e-12))
        return "Fixed boundary edge exceeds requested maximum triangle-edge "
               "length";
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
  static void prepare(MeshResult &, double) {}
  static std::optional<std::string> conflict(const MeshResult &, double) {
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
  static void prepare(MeshResult &r, double h) {
    if (!(h > 0))
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
  static std::optional<std::string> conflict(const MeshResult &, double) {
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
    s.topology->initialize_size(s.target);
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
