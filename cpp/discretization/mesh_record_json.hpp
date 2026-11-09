#pragma once
// Versioned JSON of mesh run records (MeshRunRecord, mesh_types.hpp), with
// the conventions of the FEM run records (fem/record_json.hpp): stable keys,
// classic-locale numbers with max_digits10 digits (cpp/json.hpp), null for
// absent values, a schema name and version.
//
//   write_json(out, result.record);            // schema
//   "pde2d.mesh_run_record" auto doc = read_mesh_run_record_json(in);  //
//   checks schema + version
//
// The mesh options and report encodings here are the ones the FEM run
// record uses for a mesh's provenance (record_json.hpp calls them), and the
// FEM record embeds the whole mesh run record under
// "mesh.provenance.run". Adding a key keeps the schema version; renaming or
// removing one increments it. Size fields and criteria are callables:
// only whether they are set and their descriptors are written.
#include "../json.hpp"
#include "mesh_types.hpp"
#include "provenance.hpp"
#include <istream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::discretization {
inline constexpr const char *mesh_run_record_schema = "pde2d.mesh_run_record";
inline constexpr int mesh_run_record_schema_version = 1;

// {"name", "parameters": {name: value}} or null.
inline JsonValue
descriptor_json(const std::optional<pde2d::StrategyDescriptor> &d) {
  if (!d)
    return JsonValue::null();
  auto v = JsonValue::object();
  v.set("name", json_string(d->name));
  auto parameters = JsonValue::object();
  for (const auto &p : d->parameters)
    parameters.set(p.name, json_string(p.value));
  v.set("parameters", std::move(parameters));
  return v;
}
inline JsonValue mesh_options_json(const MeshOptions &o) {
  auto boundary = JsonValue::object();
  boundary.set("max_edge_length", json_number(o.boundary.max_edge_length));
  boundary.set("max_curve_deviation",
               json_number(o.boundary.max_curve_deviation));
  boundary.set("preserve_corners", json_bool(o.boundary.preserve_corners));
  auto refinement = JsonValue::object();
  refinement.set("max_edge_length", json_number(o.refinement.max_edge_length));
  refinement.set("max_steps", json_number(o.refinement.max_steps));
  refinement.set("size_field",
                 json_bool(static_cast<bool>(o.refinement.size_field)));
  refinement.set("size_field_descriptor",
                 descriptor_json(o.refinement.size_field_descriptor));
  refinement.set("criterion", json_bool(o.refinement.criterion.has_value()));
  refinement.set("criterion_descriptor",
                 o.refinement.criterion
                     ? descriptor_json(o.refinement.criterion->descriptor)
                     : JsonValue::null());
  auto v = JsonValue::object();
  v.set("algorithm", json_string(interior_algorithm_name(o.algorithm)));
  v.set("boundary_treatment",
        json_string(boundary_treatment_name(o.boundary_treatment)));
  v.set("preserve_regions", json_bool(o.preserve_regions));
  v.set("boundary", std::move(boundary));
  v.set("refinement", std::move(refinement));
  return v;
}
inline JsonValue mesh_report_json(const MeshReport &r) {
  auto v = JsonValue::object();
  v.set("size_satisfied", json_bool(r.size_satisfied));
  v.set("delaunay_verified", json_bool(r.delaunay_verified));
  v.set("refinement_steps", json_number(r.refinement_steps));
  v.set("message", json_string(r.message));
  if (r.marked) {
    const auto &m = *r.marked;
    auto marked = JsonValue::object();
    marked.set("marked_elements", json_number(m.marked_elements));
    marked.set("marked_edges", json_number(m.marked_edges));
    marked.set("bisected_edges", json_number(m.bisected_edges));
    marked.set("absorbed_edges", json_number(m.absorbed_edges));
    marked.set("unrefinable_elements", json_number(m.unrefinable_elements));
    v.set("marked", std::move(marked));
  } else
    v.set("marked", JsonValue::null());
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
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != mesh_run_record_schema)
    throw std::invalid_argument("Not a mesh run record");
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text != std::to_string(mesh_run_record_schema_version))
    throw std::invalid_argument(
        "Unsupported mesh run record schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}
} // namespace pde2d::discretization
