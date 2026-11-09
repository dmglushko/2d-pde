#pragma once
// Mesh run records (MeshRunRecord, mesh_types.hpp): the record of the
// meshing call that produced a MeshResult. The meshing entry points
// (triangulate.hpp, mesh_io.hpp) fill MeshResult::record; this header holds
// the parts a caller may use on its own.
//
//   auto result = generate_mesh(geometry, options);
//   result.record.work.edge_flips;           // work counters of the call
//   result.record.quality.min_angle_degrees; // mesh_quality(...)
//   write_json(out, result.record);          // mesh_record_json.hpp
//
// The quality statistics are computed serially in element order, so they
// are deterministic; they read the mesh and never change it.
#include "geometry_base.hpp"
#include "mesh_fingerprint.hpp"
#include "mesh_types.hpp"
#include "provenance.hpp"
#include "size_criterion.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

namespace pde2d::discretization {
// Shape and size statistics of `mesh`. With a constant target or a size
// field in `refinement`, max_target_ratio is the largest edge length over
// its local target (local_size_target, the refinement loop's measure); a
// user RefinementCriterion has no target and leaves it unset.
inline MeshQuality mesh_quality(const TriangleMesh &mesh,
                                const RefinementOptions &refinement = {}) {
  MeshQuality q;
  q.nodes = mesh.nodes.size();
  q.elements = mesh.elements.size();
  q.boundary_edges = mesh.boundary_edges.size();
  if (mesh.elements.empty())
    return q;
  const bool targeted =
      !refinement.criterion && (refinement.max_edge_length > 0 ||
                                static_cast<bool>(refinement.size_field));
  const double degrees = 180 / std::numbers::pi;
  q.min_angle_degrees = 180;
  q.min_edge_length = std::numeric_limits<double>::infinity();
  double ratio = 0;
  for (const auto &t : mesh.elements)
    for (int k = 0; k < 3; ++k) {
      const auto a = mesh.nodes[t[k]], b = mesh.nodes[t[(k + 1) % 3]],
                 c = mesh.nodes[t[(k + 2) % 3]];
      const double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - a.x,
                   vy = c.y - a.y;
      const double angle =
          std::atan2(std::abs(ux * vy - uy * vx), ux * vx + uy * vy) * degrees;
      q.min_angle_degrees = std::min(q.min_angle_degrees, angle);
      q.max_angle_degrees = std::max(q.max_angle_degrees, angle);
      const double length = std::hypot(ux, uy);
      q.min_edge_length = std::min(q.min_edge_length, length);
      q.max_edge_length = std::max(q.max_edge_length, length);
      if (targeted) {
        const double target =
            refinement.size_field
                ? local_size_target(refinement.size_field,
                                    refinement.max_edge_length, a, b)
                : refinement.max_edge_length;
        ratio = std::max(ratio, length / target);
      }
    }
  if (targeted)
    q.max_target_ratio = ratio;
  return q;
}

// Complete result.record for `operation`: the produced mesh's fingerprint,
// the source mesh's (empty when there is none), the result's options, the
// library version, the quality statistics and the wall-clock time. The
// work counters are those the call accumulated (an import does none).
inline void finish_mesh_run_record(MeshResult &result, MeshOperation operation,
                                   std::string source_fingerprint,
                                   double seconds) {
  auto &r = result.record;
  r.operation = operation;
  r.fingerprint = mesh_fingerprint(result.mesh);
  r.source_fingerprint = std::move(source_fingerprint);
  r.options = result.options;
  r.library = pde2d::library_version();
  r.quality = mesh_quality(result.mesh, result.options.refinement);
  r.total_seconds = seconds;
}

namespace mesh_record_detail {
using Clock = std::chrono::steady_clock;
inline double seconds_since(Clock::time_point started) {
  return std::chrono::duration<double>(Clock::now() - started).count();
}
} // namespace mesh_record_detail
} // namespace pde2d::discretization
