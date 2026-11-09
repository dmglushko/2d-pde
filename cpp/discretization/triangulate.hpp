#pragma once
// Meshing entry points: generate_mesh, refine_mesh, subdivide_mesh,
// is_constrained_delaunay and interior_algorithm_capabilities. The private
// pieces live in detail/: ear clipping (ear_clipping.hpp), constrained
// Delaunay topology maintenance (cdt_topology.hpp), refinement on analytical
// curves (curved_boundary.hpp), the per-BoundaryTreatment policies
// (refinement_policy.hpp), the generic refinement loop (refinement_loop.hpp)
// and one unit per InteriorAlgorithm with its capabilities
// (interior_algorithms.hpp, the only dispatch on the algorithm).
//
// Every meshing call returns a MeshResult whose `record` (MeshRunRecord,
// mesh_types.hpp; mesh_record.hpp) names the operation, the fingerprints of
// the produced and the refined mesh, the options, the library version, the
// work counters, quality statistics and wall-clock time. Recording only
// reads the mesh: meshes are the same with or without it.
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include "detail/cdt_topology.hpp"
#include "detail/interior_algorithms.hpp"
#include "detail/refinement_loop.hpp"
#include "detail/refinement_policy.hpp"
#include "mesh_fingerprint.hpp"
#include "mesh_record.hpp"
#include <optional>
#include <string>
#include <utility>

namespace pde2d::discretization {

inline bool is_constrained_delaunay(const TriangleMesh &mesh) {
  validate_mesh_geometry(mesh);
  return mesh_detail::cdt_verified(mesh);
}
// What `algorithm` can do (size target, refinement, Delaunay meshes,
// required context). Throws std::invalid_argument for a value that names no
// algorithm.
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
  const double seconds = mesh_record_detail::seconds_since(started);
  finish_mesh_run_record(result, operation, std::move(source), seconds);
  return result;
}
} // namespace mesh_detail
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
  auto result = mesh_detail::refine_with_algorithm(
      MeshResult{mesh, std::move(context), options, {}}, options);
  return mesh_detail::recorded(std::move(result), MeshOperation::Refine,
                               mesh_fingerprint(mesh), started);
}
[[nodiscard]] inline MeshResult refine_mesh(const MeshResult &previous,
                                            const MeshOptions &options) {
  return refine_mesh(previous.mesh, options, previous.boundary_context);
}
[[nodiscard]] inline MeshResult
subdivide_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
               std::optional<BoundaryContext> context = std::nullopt) {
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves)
    throw std::invalid_argument("Subdivision preserves the polygon; curve "
                                "updates require Delaunay refinement");
  const auto started = mesh_record_detail::Clock::now();
  auto result = mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, true);
  return mesh_detail::recorded(std::move(result), MeshOperation::Subdivide,
                               mesh_fingerprint(mesh), started);
}
} // namespace pde2d::discretization
