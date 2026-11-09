#pragma once

#include "curve_geometry.hpp"
#include "provenance.hpp"
#include "size_criterion.hpp"
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
// SizeField (a local target length) is declared in size_criterion.hpp.
struct RefinementOptions {
  double max_edge_length = 0.0; // Zero disables the constant target.
  int max_steps = 10000;
  // Optional graded target. With max_edge_length > 0 as well, each edge must
  // meet both (the smaller target applies). Either one enables refinement.
  SizeField size_field{};
  // What `size_field` is, for run records and mesh plans (a callable cannot
  // be compared or serialized). Without it a size field is recorded as "not
  // replayable". Request-time like the field itself: files do not store it.
  std::optional<pde2d::StrategyDescriptor> size_field_descriptor{};
  // A user refinement criterion (size_criterion.hpp), for example a
  // problem-aware or anisotropic length test. It replaces max_edge_length
  // and size_field, which must then be unset; its descriptor names it in
  // records. Request-time like the size field: files do not store it.
  std::optional<RefinementCriterion> criterion{};
};
inline bool size_refinement_requested(const RefinementOptions &o) {
  return o.max_edge_length > 0 || static_cast<bool>(o.size_field) ||
         o.criterion.has_value();
}
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
  // Size refinement during generate_mesh: a constant max_edge_length, a
  // SizeField, a user RefinementCriterion.
  bool supports_size_target = false;
  bool supports_size_field = false;
  bool supports_size_criterion = false;
  // refine_mesh(mesh | result, options) and refine_mesh(result, marked, ...).
  bool supports_refinement = false;
  bool supports_marked_refinement = false;
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
  if (o.refinement.criterion) {
    if (!o.refinement.criterion->active())
      throw std::invalid_argument("Refinement criterion has no size test");
    if (o.refinement.max_edge_length > 0 || o.refinement.size_field)
      throw std::invalid_argument(
          "A refinement criterion replaces max_edge_length and size_field; "
          "combine them in the criterion (strictest_of)");
  }
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
// What produced a MeshResult (MeshRunRecord::operation). Stable names:
// mesh_operation_name (mesh run record JSON).
enum class MeshOperation {
  None, // a hand-built MeshResult: no meshing call filled the record
  Generate,
  Refine,
  MarkedRefine,
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
  case MeshOperation::MarkedRefine:
    return "marked refine";
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
  double min_edge_length = 0, max_edge_length = 0;
  // Largest edge length over its local target (the constant
  // max_edge_length and/or the size field of the refinement options, as
  // the refinement loop measures it). Unset without such a target, in
  // particular for a user RefinementCriterion, whose test is opaque.
  std::optional<double> max_target_ratio{};
  bool operator==(const MeshQuality &) const = default;
};
// The reproducibility record of the meshing call that produced a
// MeshResult (MeshResult::record). Filled by generate_mesh, refine_mesh,
// subdivide_mesh and import_mesh_result; the FEM run record embeds it
// (fem/record.hpp, MeshProvenance::run). Request-time like the size field:
// .pde2d files do not store it. Two identical calls differ only in
// `total_seconds`.
struct MeshRunRecord {
  MeshOperation operation = MeshOperation::None;
  // discretization::mesh_fingerprint of the produced mesh, and of the mesh
  // that was refined (empty for generate and import).
  std::string fingerprint{}, source_fingerprint{};
  // The options of the call; size field and criterion are recorded by their
  // descriptors (mesh_record_json.hpp).
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
