#pragma once

#include "curve_geometry.hpp"
#include <optional>

namespace discretization {

enum class InteriorAlgorithm { EarClipping, ConstrainedDelaunay };
enum class BoundaryTreatment { Fixed, SplitPolygonEdges, RefineOnCurves };

struct BoundaryOptions {
  double max_edge_length = 1.0;
  double max_curve_deviation = 0.02;
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
  std::vector<Point2> polygon_snapshot;
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
} // namespace discretization
