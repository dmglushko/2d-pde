#pragma once
// Meshing entry points: generate_mesh, refine_mesh, subdivide_mesh and
// is_constrained_delaunay. The private pieces live in detail/: ear clipping
// (ear_clipping.hpp), constrained Delaunay topology maintenance
// (cdt_topology.hpp) and refinement on analytical curves
// (curved_boundary.hpp). refine_result() below drives size refinement.
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include "detail/cdt_topology.hpp"
#include "detail/curved_boundary.hpp"
#include "detail/ear_clipping.hpp"
#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>

namespace pde2d::discretization {
namespace mesh_detail {
inline MeshResult refine_result(MeshResult result, const MeshOptions &options,
                                bool subdivision) {
  validate_options(options);
  validate_mesh_geometry(result.mesh);
  if (result.boundary_context)
    validate_boundary_context(result.mesh, *result.boundary_context,
                              options.preserve_regions,
                              options.boundary.preserve_corners);
  result.options = options;
  result.report = {};
  if (!options.preserve_regions)
    result.mesh.boundary_tags.clear();
  else if (result.mesh.boundary_tags.empty() && result.boundary_context) {
    for (const auto &spans : result.boundary_context->edge_sources) {
      auto name = result.boundary_context->curves[spans.front().curve].region;
      for (auto span : spans)
        if (result.boundary_context->curves[span.curve].region != name)
          throw std::invalid_argument(
              "Enabling regions requires regenerating boundary discretization "
              "at region junctions");
      result.mesh.boundary_tags.push_back(name);
    }
  }
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves &&
      !result.boundary_context)
    throw std::invalid_argument(
        "Curve refinement requires original boundary context");
  bool delaunay = !subdivision;
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves &&
      options.refinement.max_edge_length > 0) {
    bool off_curve = false;
    for (std::size_t i = 0; i < result.mesh.boundary_edges.size(); ++i) {
      auto s = result.boundary_context->edge_sources[i].front();
      auto p =
          curvePoint(result.boundary_context->curves[s.curve].segment, s.begin);
      if (distance(p, result.mesh.nodes[result.mesh.boundary_edges[i].first]) >
          0)
        off_curve = true;
    }
    if (off_curve)
      rebuild_curved(result);
  }
  auto topology = std::make_unique<Topology>(result.mesh);
  if (delaunay)
    topology->legalize();
  double h = options.refinement.max_edge_length;
  if (h > 0 && options.boundary_treatment == BoundaryTreatment::Fixed) {
    for (auto [a, b] : result.mesh.boundary_edges)
      if (distance(result.mesh.nodes[a], result.mesh.nodes[b]) >
          h * (1 + 1e-12)) {
        result.report.size_satisfied = false;
        result.report.delaunay_verified = delaunay;
        result.report.message = "Fixed boundary edge exceeds requested maximum "
                                "triangle-edge length";
        return result;
      }
  }
  topology->initialize_size(h);
  for (int step = 0; h > 0; ++step) {
    auto candidate = topology->longest();
    if (!candidate)
      break;
    Edge longest = *candidate;
    if (step >= options.refinement.max_steps) {
      result.report.size_satisfied = false;
      result.report.message = "Refinement step budget exhausted";
      break;
    }
    Point2 a = result.mesh.nodes[longest.first],
           b = result.mesh.nodes[longest.second];
    Point2 mid{std::midpoint(a.x, b.x), std::midpoint(a.y, b.y)};
    if ((mid.x == a.x && mid.y == a.y) || (mid.x == b.x && mid.y == b.y)) {
      result.report.size_satisfied = false;
      result.report.message = "Refinement reached coordinate precision limit";
      break;
    }
    auto it = result.mesh.boundary_edges.end();
    if (topology->constrained.contains(longest))
      it = std::find_if(
          result.mesh.boundary_edges.begin(), result.mesh.boundary_edges.end(),
          [&](auto e) { return edge_key(e.first, e.second) == longest; });
    bool boundary = it != result.mesh.boundary_edges.end();
    auto index =
        static_cast<std::size_t>(it - result.mesh.boundary_edges.begin());
    if (boundary &&
        options.boundary_treatment == BoundaryTreatment::RefineOnCurves) {
      // A temporary vertex supplies connectivity; rebuild samples its actual
      // curve parameter and does not reuse its midpoint coordinates.
      auto n = result.mesh.nodes.size();
      result.mesh.nodes.push_back(mid);
      split_boundary_record(result, index, n);
      rebuild_curved(result);
      topology = std::make_unique<Topology>(result.mesh);
      topology->initialize_size(h);
    } else {
      auto n = result.mesh.nodes.size();
      topology->split(longest, mid);
      if (boundary)
        split_boundary_record(result, index, n);
      if (delaunay)
        topology->legalize(false);
    }
    ++result.report.refinement_steps;
  }
  result.report.delaunay_verified = delaunay;
  validate_mesh_geometry(result.mesh);
  return result;
}
} // namespace mesh_detail

inline bool is_constrained_delaunay(const TriangleMesh &mesh) {
  validate_mesh_geometry(mesh);
  return mesh_detail::cdt_verified(mesh);
}
inline MeshResult generate_mesh(const DiscretizedBoundary &boundary,
                                const MeshOptions &options = {}) {
  mesh_detail::validate_options(options);
  MeshResult result;
  result.mesh = mesh_detail::seed_mesh(boundary.boundary);
  result.boundary_context = boundary.context;
  result.options = options;
  if (!options.preserve_regions)
    result.mesh.boundary_tags.clear();
  if (options.algorithm == InteriorAlgorithm::ConstrainedDelaunay)
    return mesh_detail::refine_result(std::move(result), options, false);
  validate_mesh_geometry(result.mesh);
  if (result.boundary_context)
    validate_boundary_context(result.mesh, *result.boundary_context,
                              options.preserve_regions,
                              options.boundary.preserve_corners);
  return result;
}
inline MeshResult generate_mesh(const Boundary &boundary,
                                const MeshOptions &options = {}) {
  return generate_mesh(DiscretizedBoundary{boundary, std::nullopt}, options);
}
inline MeshResult generate_mesh(const CurveGeometry &geometry,
                                const MeshOptions &options = {}) {
  mesh_detail::validate_options(options);
  return generate_mesh(
      discretize_boundary(geometry, options.boundary, options.preserve_regions),
      options);
}
inline MeshResult
refine_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
            std::optional<BoundaryContext> context = std::nullopt) {
  if (options.algorithm != InteriorAlgorithm::ConstrainedDelaunay)
    throw std::invalid_argument("refine_mesh uses constrained Delaunay; use "
                                "subdivide_mesh for subdivision");
  return mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, false);
}
inline MeshResult refine_mesh(const MeshResult &previous,
                              const MeshOptions &options) {
  return refine_mesh(previous.mesh, options, previous.boundary_context);
}
inline MeshResult
subdivide_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
               std::optional<BoundaryContext> context = std::nullopt) {
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves)
    throw std::invalid_argument("Subdivision preserves the polygon; curve "
                                "updates require Delaunay refinement");
  return mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, true);
}
} // namespace pde2d::discretization
