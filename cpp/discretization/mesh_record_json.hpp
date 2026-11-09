#pragma once
// Versioned JSON of mesh run records (MeshRunRecord, mesh_types.hpp), with
// the conventions of the FEM run records (fem/record_json.hpp): stable keys,
// classic-locale numbers with max_digits10 digits (cpp/json.hpp), null for
// absent values, a schema name and version.
//
//   write_json(out, result.record);   // schema "pde2d.mesh_run_record"
//   auto doc = read_mesh_run_record_json(in);   // checks schema + version
//   auto diff = diff_records(result.record, other.record);  // no timings
//
// The FEM run record embeds this document under "mesh.run" when a mesh run
// record is attached (fem/record.hpp). Adding a key keeps the schema
// version; renaming or removing one, or changing a value's meaning,
// increments it. Keys are listed in cpp/discretization/README.md.
#include "../json.hpp"
#include "mesh_types.hpp"
#include <istream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::discretization {
inline constexpr const char *mesh_run_record_schema = "pde2d.mesh_run_record";
inline constexpr int mesh_run_record_schema_version = 1;

inline JsonValue mesh_options_json(const MeshOptions &o) {
  auto boundary = JsonValue::object();
  boundary.set("max_edge_length", json_number(o.boundary.max_edge_length));
  boundary.set("max_curve_deviation",
               json_number(o.boundary.max_curve_deviation));
  boundary.set("preserve_corners", json_bool(o.boundary.preserve_corners));
  auto refinement = JsonValue::object();
  refinement.set("max_edge_length", json_number(o.refinement.max_edge_length));
  refinement.set("max_steps", json_number(o.refinement.max_steps));
  auto v = JsonValue::object();
  v.set("algorithm", json_string(interior_algorithm_name(o.algorithm)));
  v.set("boundary_treatment",
        json_string(boundary_treatment_name(o.boundary_treatment)));
  v.set("preserve_regions", json_bool(o.preserve_regions));
  v.set("boundary", std::move(boundary));
  v.set("refinement", std::move(refinement));
  return v;
}

// The JSON document of a mesh run record (schema mesh_run_record_schema,
// version mesh_run_record_schema_version).
inline JsonValue to_json(const MeshRunRecord &r) {
  auto v = JsonValue::object();
  v.set("schema", json_string(mesh_run_record_schema));
  v.set("schema_version", json_number(mesh_run_record_schema_version));
  v.set("operation", json_string(mesh_operation_name(r.operation)));
  v.set("fingerprint", json_string(r.fingerprint));
  v.set("source_fingerprint", r.source_fingerprint.empty()
                                  ? JsonValue::null()
                                  : json_string(r.source_fingerprint));
  auto versions = JsonValue::object();
  versions.set("pde2d", json_string(r.library.version));
  versions.set("git_sha", json_string(r.library.git_sha));
  v.set("versions", std::move(versions));
  v.set("options", mesh_options_json(r.options));
  const auto &w = r.work;
  auto work = JsonValue::object();
  work.set("refinement_steps", json_number(w.refinement_steps));
  work.set("inserted_vertices", json_number(w.inserted_vertices));
  work.set("boundary_splits", json_number(w.boundary_splits));
  work.set("edge_flips", json_number(w.edge_flips));
  work.set("curved_rebuilds", json_number(w.curved_rebuilds));
  work.set("reinserted_vertices", json_number(w.reinserted_vertices));
  work.set("removed_vertices", json_number(w.removed_vertices));
  v.set("work", std::move(work));
  const auto &q = r.quality;
  auto quality = JsonValue::object();
  quality.set("nodes", json_number(q.nodes));
  quality.set("elements", json_number(q.elements));
  quality.set("boundary_edges", json_number(q.boundary_edges));
  quality.set("min_angle_degrees", json_number(q.min_angle_degrees));
  quality.set("max_angle_degrees", json_number(q.max_angle_degrees));
  quality.set("min_edge_length", json_number(q.min_edge_length));
  quality.set("max_edge_length", json_number(q.max_edge_length));
  quality.set("max_target_ratio", q.max_target_ratio
                                      ? json_number(*q.max_target_ratio)
                                      : JsonValue::null());
  v.set("quality", std::move(quality));
  auto timings = JsonValue::object();
  timings.set("total_seconds", json_number(r.total_seconds));
  v.set("timings", std::move(timings));
  return v;
}
inline std::ostream &write_json(std::ostream &out, const MeshRunRecord &r) {
  return pde2d::write_json(out, to_json(r));
}

// Parse a mesh run record document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a mesh run record",
// "Unsupported mesh run record schema version: N").
inline JsonValue read_mesh_run_record_json(std::istream &in) {
  return json_detail::read_schema(in, mesh_run_record_schema,
                                  mesh_run_record_schema_version,
                                  "mesh run record");
}

// Leaves of the two documents that differ (pde2d::diff_records; timings
// are ignored by default).
inline std::vector<RecordDifference>
diff_records(const MeshRunRecord &left, const MeshRunRecord &right,
             const RecordDiffOptions &options = {}) {
  return pde2d::diff_records(to_json(left), to_json(right), options);
}
} // namespace pde2d::discretization
