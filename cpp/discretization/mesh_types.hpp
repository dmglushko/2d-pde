#pragma once

#include "curve_geometry.hpp"
#include "provenance.hpp"
#include <optional>
#include <string>

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
inline const char *boundary_treatment_name(BoundaryTreatment value) {
  switch (value) {
  case BoundaryTreatment::Fixed:
    return "Fixed";
  case BoundaryTreatment::SplitPolygonEdges:
    return "SplitPolygonEdges";
  case BoundaryTreatment::RefineOnCurves:
    return "RefineOnCurves";
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
// What produced a MeshResult (MeshRunRecord::operation). Stable names:
// mesh_operation_name (mesh run record JSON).
enum class MeshOperation {
  None, // a hand-built MeshResult: no meshing call filled the record
  Generate,
  Refine,
  Subdivide,
  Import
};
inline const char *mesh_operation_name(MeshOperation operation) {
  switch (operation) {
  case MeshOperation::None:
    return "none";
  case MeshOperation::Generate:
    return "generate";
  case MeshOperation::Refine:
    return "refine";
  case MeshOperation::Subdivide:
    return "subdivide";
  case MeshOperation::Import:
    return "import";
  }
  return "invalid";
}
// Work counters of one meshing call. Counting never changes the mesh.
struct MeshWork {
  // Edge splits of the refinement loop (MeshReport::refinement_steps); each
  // inserts one vertex, so inserted_vertices == refinement_steps.
  std::size_t refinement_steps = 0, inserted_vertices = 0;
  // Of those, splits of a boundary edge (chord or curve).
  std::size_t boundary_splits = 0;
  // Delaunay edge flips of every legalization, curved rebuilds included.
  std::size_t edge_flips = 0;
  // RefineOnCurves triangulation rebuilds, the interior vertices they
  // reinserted and those they dropped (outside or on the updated boundary).
  std::size_t curved_rebuilds = 0, reinserted_vertices = 0,
              removed_vertices = 0;
  bool operator==(const MeshWork &) const = default;
};
// Shape and size statistics of a mesh (mesh_quality, mesh_record.hpp).
struct MeshQuality {
  std::size_t nodes = 0, elements = 0, boundary_edges = 0;
  // Interior angles in degrees, over every triangle (0 without triangles).
  double min_angle_degrees = 0, max_angle_degrees = 0;
  // Triangle edge lengths (0 without triangles).
  double min_edge_length = 0, max_edge_length = 0;
  // max_edge_length over RefinementOptions::max_edge_length; unset without
  // a size target. At most 1 (to rounding) when the target was met.
  std::optional<double> max_target_ratio{};
  bool operator==(const MeshQuality &) const = default;
};
// The reproducibility record of the meshing call that produced a
// MeshResult (MeshResult::record). Filled by generate_mesh, refine_mesh,
// subdivide_mesh and import_mesh_result (mesh_record.hpp); serialized by
// mesh_record_json.hpp; a FEM run record can carry it
// (fem/record.hpp, attach_mesh_record). .pde2d files do not store it. Two
// identical calls differ only in `total_seconds`.
struct MeshRunRecord {
  MeshOperation operation = MeshOperation::None;
  // discretization::mesh_fingerprint of the produced mesh, and of the mesh
  // that was refined or subdivided (empty for generate and import).
  std::string fingerprint{}, source_fingerprint{};
  MeshOptions options{};
  pde2d::LibraryVersion library{};
  MeshWork work{};
  MeshQuality quality{};
  // Wall-clock seconds of the call. Never reproducible.
  double total_seconds = 0;
};
struct MeshResult {
  TriangleMesh mesh;
  std::optional<BoundaryContext> boundary_context;
  MeshOptions options{};
  MeshReport report{};
  // What produced this result (MeshRunRecord).
  MeshRunRecord record{};
};
} // namespace pde2d::discretization
