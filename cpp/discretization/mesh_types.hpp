#pragma once

#include "curve_geometry.hpp"
#include <optional>

namespace pde2d::discretization {

// Explicit values are stable PDE2D_MESH wire IDs. Never renumber.
enum class InteriorAlgorithm { EarClipping = 0, ConstrainedDelaunay = 1 };
enum class BoundaryTreatment {
  Fixed = 0,
  SplitPolygonEdges = 1,
  RefineOnCurves = 2
};

struct BoundaryOptions {
  double max_edge_length = 1.0;
  double max_curve_deviation = 0.02;
  bool preserve_corners = false;
};
struct RefinementOptions {
  double max_edge_length = 0.0; // Zero disables size refinement.
  int max_steps = 10000;
};
struct MeshOptions {
  InteriorAlgorithm algorithm = InteriorAlgorithm::ConstrainedDelaunay;
  BoundaryTreatment boundary_treatment = BoundaryTreatment::Fixed;
  bool preserve_regions = true;
  BoundaryOptions boundary{};
  RefinementOptions refinement{};
};
// What an interior algorithm can do. Each algorithm is one unit
// (detail/interior_algorithms.hpp) declaring these; requests outside them
// throw with the unit's message. interior_algorithm_capabilities()
// (triangulate.hpp) returns them.
struct InteriorAlgorithmCapabilities {
  // Size refinement (RefinementOptions::max_edge_length) during
  // generate_mesh.
  bool supports_size_target = false;
  // refine_mesh(mesh | result, options).
  bool supports_refinement = false;
  // Its meshes are constrained Delaunay (MeshReport::delaunay_verified).
  bool constrained_delaunay = false;
  // generate_mesh needs analytical boundary context (a DiscretizedBoundary
  // from discretize_boundary, not a raw Boundary).
  bool requires_boundary_context = false;
  bool operator==(const InteriorAlgorithmCapabilities &) const = default;
};
// Report names of the enum values (not wire IDs; those are the numbers).
inline const char *interior_algorithm_name(InteriorAlgorithm value) {
  switch (value) {
  case InteriorAlgorithm::EarClipping:
    return "EarClipping";
  case InteriorAlgorithm::ConstrainedDelaunay:
    return "ConstrainedDelaunay";
  }
  return "Unknown";
}
// Shared value validation. Execution-specific compatibility (for example,
// ear clipping versus refinement) belongs to the requested operation.
inline bool validInteriorAlgorithm(InteriorAlgorithm value) {
  switch (value) {
  case InteriorAlgorithm::EarClipping:
  case InteriorAlgorithm::ConstrainedDelaunay:
    return true;
  }
  return false;
}
inline bool validBoundaryTreatment(BoundaryTreatment value) {
  switch (value) {
  case BoundaryTreatment::Fixed:
  case BoundaryTreatment::SplitPolygonEdges:
  case BoundaryTreatment::RefineOnCurves:
    return true;
  }
  return false;
}
inline void validateNonnegative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0)
    throw std::invalid_argument(
        std::string(name) +
        " must be finite and nonnegative; zero disables it");
}
inline void validateMeshOptionValues(const MeshOptions &o) {
  if (!validInteriorAlgorithm(o.algorithm))
    throw std::invalid_argument("Unknown interior algorithm");
  if (!validBoundaryTreatment(o.boundary_treatment))
    throw std::invalid_argument("Unknown boundary treatment");
  validateNonnegative(o.boundary.max_edge_length, "Boundary edge length");
  validateNonnegative(o.boundary.max_curve_deviation, "Curve deviation");
  validateNonnegative(o.refinement.max_edge_length, "Refinement edge length");
  if (o.refinement.max_steps < 0)
    throw std::invalid_argument("Refinement step budget must be nonnegative");
}
struct CurveInterval {
  std::size_t curve;
  double begin, end;
};
struct BoundaryContext {
  // World-coordinate snapshot; source IDs belong to the original geometry.
  std::vector<BoundaryCurve> curves;
  // Ordered curve intervals represented by each straight mesh boundary edge.
  std::vector<std::vector<CurveInterval>> edge_sources;
  std::vector<std::size_t> protected_vertices;
  // Polygon at sampling or the last analytical-curve update. Chord splits
  // must remain on this polygon; its vertices are anchored to source curves.
  // With holes it holds every loop, outer first, one after another.
  std::vector<Point2> polygon_snapshot;
  // Vertex counts of the consecutive loops in polygon_snapshot (outer loop
  // first). Empty means one loop: the historical single-outline snapshot.
  std::vector<std::size_t> snapshot_loop_sizes{};
};
struct MeshReport {
  bool size_satisfied = true;
  bool delaunay_verified = false;
  std::size_t refinement_steps = 0;
  std::string message;
};
struct MeshResult {
  TriangleMesh mesh;
  std::optional<BoundaryContext> boundary_context;
  MeshOptions options{};
  MeshReport report{};
};
} // namespace pde2d::discretization
