#pragma once
// Meshing entry points: generate_mesh, refine_mesh, subdivide_mesh and
// is_constrained_delaunay. The private pieces live in detail/: ear clipping
// (ear_clipping.hpp), constrained Delaunay topology maintenance
// (cdt_topology.hpp) and refinement on analytical curves
// (curved_boundary.hpp). Size criteria and the per-BoundaryTreatment
// policies are in refinement_policy.hpp; refine_with() below is the generic
// refinement loop that combines them.
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include "detail/cdt_topology.hpp"
#include "detail/curved_boundary.hpp"
#include "detail/ear_clipping.hpp"
#include "detail/refinement_policy.hpp"
#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace pde2d::discretization {
namespace mesh_detail {
// Region tags: cleared when regions are off, otherwise restored from the
// boundary context if the mesh lost them.
inline void restore_region_tags(MeshResult &result,
                                const MeshOptions &options) {
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
}
// The generic loop: split the highest-priority edge failing the criterion
// until none is left, the step budget runs out or coordinates stop
// resolving. Policy decides everything that depends on the boundary
// treatment.
template <class Policy>
MeshResult refine_with(MeshResult result, const MeshOptions &options,
                       bool delaunay, const RefinementCriterion &criterion) {
  Policy::prepare(result, criterion);
  RefinementState state{result, std::make_unique<Topology>(result.mesh),
                        criterion, delaunay};
  if (delaunay)
    state.topology->legalize();
  if (criterion.active())
    if (auto conflict = Policy::conflict(result, criterion)) {
      result.report.size_satisfied = false;
      result.report.delaunay_verified = delaunay;
      result.report.message = *conflict;
      return result;
    }
  state.topology->initialize_size(criterion.size);
  for (int step = 0; criterion.active(); ++step) {
    auto candidate = state.topology->longest();
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
    if (state.topology->constrained.contains(longest))
      it = std::find_if(
          result.mesh.boundary_edges.begin(), result.mesh.boundary_edges.end(),
          [&](auto e) { return edge_key(e.first, e.second) == longest; });
    if (it != result.mesh.boundary_edges.end()) {
      auto index =
          static_cast<std::size_t>(it - result.mesh.boundary_edges.begin());
      Policy::split_boundary(state, longest, index, mid);
    } else {
      state.topology->split(longest, mid);
      if (delaunay)
        state.topology->legalize(false);
    }
    if (criterion.split_hook)
      criterion.split_hook(a, b);
    ++result.report.refinement_steps;
  }
  result.report.delaunay_verified = delaunay;
  validate_mesh_geometry(result.mesh);
  return result;
}
inline MeshResult refine_result(MeshResult result, const MeshOptions &options,
                                bool subdivision,
                                const RefinementCriterion &criterion) {
  validate_options(options);
  validate_mesh_geometry(result.mesh);
  if (result.boundary_context)
    validate_boundary_context(result.mesh, *result.boundary_context,
                              options.preserve_regions,
                              options.boundary.preserve_corners);
  result.options = options;
  result.report = {};
  restore_region_tags(result, options);
  return with_boundary_policy(
      options.boundary_treatment, [&](auto policy) -> MeshResult {
        using Policy = decltype(policy);
        Policy::validate(result);
        return refine_with<Policy>(std::move(result), options, !subdivision,
                                   criterion);
      });
}
inline MeshResult refine_result(MeshResult result, const MeshOptions &options,
                                bool subdivision) {
  return refine_result(std::move(result), options, subdivision,
                       size_criterion(options.refinement));
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
// Marked refinement for adaptivity: bisect the longest edge of every marked
// triangle of previous.mesh (indices into previous.mesh.elements; duplicates
// are ignored) with the CDT machinery, so neighbours across a bisected edge
// are split too and the mesh stays conforming and Delaunay. The boundary
// treatment decides boundary edges: Fixed selects the longest interior edge
// instead, SplitPolygonEdges splits the chord, RefineOnCurves places the new
// vertex on the source curve. A size target in options (max_edge_length or
// size_field) is enforced afterwards. report.marked says what happened;
// report.size_satisfied is false if a marked triangle could not be refined,
// the step budget ran out or the size target was missed.
inline MeshResult refine_mesh(const MeshResult &previous,
                              const std::vector<std::size_t> &marked,
                              const MeshOptions &options) {
  if (options.algorithm != InteriorAlgorithm::ConstrainedDelaunay)
    throw std::invalid_argument("refine_mesh uses constrained Delaunay; use "
                                "subdivide_mesh for subdivision");
  validate_mesh_geometry(previous.mesh);
  auto state = mesh_detail::select_marked_edges(previous.mesh, marked,
                                                options.boundary_treatment !=
                                                    BoundaryTreatment::Fixed);
  auto result = mesh_detail::refine_result(
      MeshResult{previous.mesh, previous.boundary_context, options, {}},
      options, false,
      mesh_detail::marked_criterion(
          state, mesh_detail::size_criterion(options.refinement)));
  MarkedRefinementReport report;
  report.marked_elements = state->marked_elements;
  report.marked_edges = state->marked_edges;
  report.bisected_edges = state->bisected;
  report.unrefinable_elements = state->unrefinable;
  // A selected edge still pending after the loop either vanished before its
  // turn (absorbed) or was left over because the loop stopped early.
  std::set<mesh_detail::CoordinateEdge> present;
  for (const auto &t : result.mesh.elements)
    for (int e = 0; e < 3; ++e)
      present.insert(mesh_detail::coordinate_edge(
          result.mesh.nodes[t[e]], result.mesh.nodes[t[(e + 1) % 3]]));
  for (const auto &[edge, tier] : state->pending) {
    (void)tier;
    if (!present.contains(edge))
      ++report.absorbed_edges;
  }
  if (report.unrefinable_elements > 0) {
    result.report.size_satisfied = false;
    std::string note = std::to_string(report.unrefinable_elements) +
                       " marked triangles have no edge the boundary "
                       "treatment allows splitting";
    result.report.message = result.report.message.empty()
                                ? note
                                : result.report.message + "; " + note;
  }
  result.report.marked = report;
  return result;
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
