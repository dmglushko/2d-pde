#pragma once
// Meshing entry points: generate_mesh, refine_mesh, subdivide_mesh,
// is_constrained_delaunay and interior_algorithm_capabilities. The private
// pieces live in detail/: ear clipping (ear_clipping.hpp), constrained
// Delaunay topology maintenance (cdt_topology.hpp), refinement on analytical
// curves (curved_boundary.hpp), the per-BoundaryTreatment policies
// (refinement_policy.hpp), the generic refinement loop (refinement_loop.hpp)
// and one unit per InteriorAlgorithm with its capabilities
// (interior_algorithms.hpp, the only dispatch on the algorithm). Size
// criteria are public: size_criterion.hpp.
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include "detail/cdt_topology.hpp"
#include "detail/interior_algorithms.hpp"
#include "detail/refinement_loop.hpp"
#include "detail/refinement_policy.hpp"
#include "mesh_fingerprint.hpp"
#include "mesh_record.hpp"
#include "size_criterion.hpp"
#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace pde2d::discretization {

inline bool is_constrained_delaunay(const TriangleMesh &mesh) {
  validate_mesh_geometry(mesh);
  return mesh_detail::cdt_verified(mesh);
}
// What `algorithm` can do (size targets, size fields and criteria,
// refinement, marked refinement, Delaunay meshes, required context).
// Throws std::invalid_argument for a value that names no algorithm.
inline InteriorAlgorithmCapabilities
interior_algorithm_capabilities(InteriorAlgorithm algorithm) {
  return mesh_detail::with_interior_algorithm(
      algorithm, [](auto unit) { return decltype(unit)::capabilities; });
}
namespace mesh_detail {
// The result of a meshing call with its record (mesh_record.hpp).
inline MeshResult recorded(MeshResult result, MeshOperation operation,
                           std::string source,
                           mesh_record_detail::Clock::time_point started) {
  finish_mesh_run_record(result, operation, std::move(source),
                         mesh_record_detail::seconds_since(started));
  return result;
}
} // namespace mesh_detail
// Every meshing call returns a MeshResult whose `record` (MeshRunRecord,
// mesh_types.hpp) names the operation, the fingerprints of the produced and
// the refined mesh, the options, the library version, the work counters,
// quality statistics and wall-clock time.
[[nodiscard]] inline MeshResult
generate_mesh(const DiscretizedBoundary &boundary,
              const MeshOptions &options = {}) {
  const auto started = mesh_record_detail::Clock::now();
  return mesh_detail::recorded(
      mesh_detail::generate_with_algorithm(boundary, options),
      MeshOperation::Generate, {}, started);
}
[[nodiscard]] inline MeshResult generate_mesh(const Boundary &boundary,
                                              const MeshOptions &options = {}) {
  return generate_mesh(DiscretizedBoundary{boundary, std::nullopt}, options);
}
[[nodiscard]] inline MeshResult generate_mesh(const CurveGeometry &geometry,
                                              const MeshOptions &options = {}) {
  const auto started = mesh_record_detail::Clock::now();
  mesh_detail::validate_options(options);
  // The record's time includes boundary sampling.
  return mesh_detail::recorded(
      mesh_detail::generate_with_algorithm(
          discretize_boundary(geometry, options.boundary,
                              options.preserve_regions),
          options),
      MeshOperation::Generate, {}, started);
}
[[nodiscard]] inline MeshResult
refine_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
            std::optional<BoundaryContext> context = std::nullopt) {
  const auto started = mesh_record_detail::Clock::now();
  mesh_detail::require_refinement(options.algorithm, false);
  return mesh_detail::recorded(
      mesh_detail::refine_with_algorithm(
          MeshResult{mesh, std::move(context), options, {}}, options,
          mesh_detail::size_criterion(options.refinement), false),
      MeshOperation::Refine, mesh_fingerprint(mesh), started);
}
[[nodiscard]] inline MeshResult refine_mesh(const MeshResult &previous,
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
[[nodiscard]] inline MeshResult
refine_mesh(const MeshResult &previous, const std::vector<std::size_t> &marked,
            const MeshOptions &options) {
  const auto started = mesh_record_detail::Clock::now();
  mesh_detail::require_refinement(options.algorithm, true);
  validate_mesh_geometry(previous.mesh);
  auto state = mesh_detail::select_marked_edges(previous.mesh, marked,
                                                options.boundary_treatment !=
                                                    BoundaryTreatment::Fixed);
  auto result = mesh_detail::refine_with_algorithm(
      MeshResult{previous.mesh, previous.boundary_context, options, {}},
      options,
      mesh_detail::marked_criterion(
          state, mesh_detail::size_criterion(options.refinement)),
      true);
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
  return mesh_detail::recorded(std::move(result), MeshOperation::MarkedRefine,
                               mesh_fingerprint(previous.mesh), started);
}
[[nodiscard]] inline MeshResult
subdivide_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
               std::optional<BoundaryContext> context = std::nullopt) {
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves)
    throw std::invalid_argument("Subdivision preserves the polygon; curve "
                                "updates require Delaunay refinement");
  const auto started = mesh_record_detail::Clock::now();
  return mesh_detail::recorded(
      mesh_detail::refine_result(
          MeshResult{mesh, std::move(context), options, {}}, options, true),
      MeshOperation::Subdivide, mesh_fingerprint(mesh), started);
}
} // namespace pde2d::discretization
