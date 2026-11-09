#pragma once
// Private (mesh_detail) part of triangulate.hpp: the generic refinement loop.
// refine_with() splits the highest-priority edge failing the criterion
// (size_criterion.hpp) until none is left; the boundary policy of the
// BoundaryTreatment (refinement_policy.hpp) decides everything that depends
// on the boundary. refine_result() validates a request and dispatches the
// policy. Interior algorithms (interior_algorithms.hpp) call it; it does no
// algorithm dispatch itself.
#include "../boundary_context.hpp"
#include "../boundary_discretization.hpp"
#include "../size_criterion.hpp"
#include "cdt_topology.hpp"
#include "curved_boundary.hpp"
#include "ear_clipping.hpp"
#include "refinement_policy.hpp"
#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::discretization::mesh_detail {
// Value and algorithm-compatibility checks of meshing options; defined with
// the interior algorithm units (interior_algorithms.hpp), whose capabilities
// decide compatibility.
inline void validate_options(const MeshOptions &o);

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
  state.topology->work = &result.record.work;
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
      ++result.record.work.boundary_splits;
    } else {
      state.topology->split(longest, mid);
      if (delaunay)
        state.topology->legalize(false);
    }
    if (criterion.split_hook)
      criterion.split_hook(a, b);
    ++result.report.refinement_steps;
    ++result.record.work.inserted_vertices;
  }
  result.record.work.refinement_steps = result.report.refinement_steps;
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
  result.record = {};
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
} // namespace pde2d::discretization::mesh_detail
