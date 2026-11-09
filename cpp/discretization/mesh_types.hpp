#pragma once

#include "curve_geometry.hpp"
#include <functional>
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
// Local target edge length at a point, for spatially graded meshes. An edge
// (a, b) meets the field when its length is at most
// min(h(a), h((a+b)/2), h(b)); values must be finite and positive. A size
// field is a request-time option: .pde2d files do not store it (a callable
// cannot be serialized), so an imported result has an empty field.
using SizeField = std::function<double(Point2)>;
struct RefinementOptions {
  double max_edge_length = 0.0; // Zero disables the constant target.
  int max_steps = 10000;
  // Optional graded target. With max_edge_length > 0 as well, each edge must
  // meet both (the smaller target applies). Either one enables refinement.
  SizeField size_field{};
};
inline bool size_refinement_requested(const RefinementOptions &o) {
  return o.max_edge_length > 0 || static_cast<bool>(o.size_field);
}
struct MeshOptions {
  InteriorAlgorithm algorithm = InteriorAlgorithm::ConstrainedDelaunay;
  BoundaryTreatment boundary_treatment = BoundaryTreatment::Fixed;
  bool preserve_regions = true;
  BoundaryOptions boundary{};
  RefinementOptions refinement{};
};
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
// What refine_mesh(result, marked, options) did with the marked triangles.
// Request-time detail: .pde2d files do not store it.
struct MarkedRefinementReport {
  std::size_t marked_elements = 0; // distinct marked triangles
  std::size_t marked_edges = 0;    // distinct edges selected for bisection
  std::size_t bisected_edges = 0;  // selected edges that were split
  // Selected edges that an earlier insertion removed (a Delaunay flip or a
  // curved-boundary rebuild) before their turn: their triangle was already
  // changed by a nearby vertex.
  std::size_t absorbed_edges = 0;
  // Triangles with no edge the boundary treatment allows splitting (Fixed:
  // every edge on the boundary).
  std::size_t unrefinable_elements = 0;
};
struct MeshReport {
  bool size_satisfied = true;
  bool delaunay_verified = false;
  std::size_t refinement_steps = 0;
  std::string message;
  std::optional<MarkedRefinementReport> marked{};
};
struct MeshResult {
  TriangleMesh mesh;
  std::optional<BoundaryContext> boundary_context;
  MeshOptions options{};
  MeshReport report{};
};
} // namespace pde2d::discretization
