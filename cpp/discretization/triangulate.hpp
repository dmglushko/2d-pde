#pragma once
// Meshing entry points: generate_mesh, refine_mesh, subdivide_mesh,
// is_constrained_delaunay and interior_algorithm_capabilities. The private
// pieces live in detail/: ear clipping (ear_clipping.hpp), constrained
// Delaunay topology maintenance (cdt_topology.hpp), refinement on analytical
// curves (curved_boundary.hpp), the per-BoundaryTreatment policies
// (refinement_policy.hpp), the generic refinement loop (refinement_loop.hpp)
// and one unit per InteriorAlgorithm with its capabilities
// (interior_algorithms.hpp, the only dispatch on the algorithm).
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include "detail/cdt_topology.hpp"
#include "detail/interior_algorithms.hpp"
#include "detail/refinement_loop.hpp"
#include "detail/refinement_policy.hpp"
#include <optional>

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
[[nodiscard]] inline MeshResult
generate_mesh(const DiscretizedBoundary &boundary,
              const MeshOptions &options = {}) {
  return mesh_detail::generate_with_algorithm(boundary, options);
}
[[nodiscard]] inline MeshResult generate_mesh(const Boundary &boundary,
                                              const MeshOptions &options = {}) {
  return generate_mesh(DiscretizedBoundary{boundary, std::nullopt}, options);
}
[[nodiscard]] inline MeshResult generate_mesh(const CurveGeometry &geometry,
                                              const MeshOptions &options = {}) {
  mesh_detail::validate_options(options);
  return mesh_detail::generate_with_algorithm(
      discretize_boundary(geometry, options.boundary, options.preserve_regions),
      options);
}
[[nodiscard]] inline MeshResult
refine_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
            std::optional<BoundaryContext> context = std::nullopt) {
  return mesh_detail::refine_with_algorithm(
      MeshResult{mesh, std::move(context), options, {}}, options);
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
  return mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, true);
}
} // namespace pde2d::discretization
