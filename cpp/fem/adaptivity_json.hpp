#pragma once
// Versioned JSON of an adaptive loop (adaptivity.hpp), with the conventions
// of run records (record_json.hpp): stable keys, classic-locale numbers with
// max_digits10 digits, null for absent values.
//
//   write_json(out, history);                  // schema
//   "pde2d.adaptive_history" auto doc = read_adaptive_history_json(in); //
//   checks schema + version diff_records(to_json(a), to_json(b));      // leaf
//   by leaf
//
// Every iteration embeds the full run record of its solve under "record"
// (schema "pde2d.run_record"). Keys are documented in docs/fem/README.md
// ("Adaptive loop"); adding a key keeps the schema version, renaming or
// removing one increments it. Indicators and nodal values are not written.
#include "fem/adaptivity.hpp"
#include "fem/json.hpp"
#include "fem/record_json.hpp"
#include <istream>
#include <ostream>
#include <string>

namespace pde2d::fem {
inline constexpr const char *adaptive_history_schema = "pde2d.adaptive_history";
inline constexpr int adaptive_history_schema_version = 1;

namespace adaptivity_json_detail {
inline JsonValue estimate(const EstimateSummary &e) {
  auto v = JsonValue::object();
  v.set("estimator", json_string(e.estimator));
  v.set("descriptor", record_json_detail::descriptor(e.descriptor));
  v.set("global", json_number(e.global));
  v.set("elements", json_number(e.elements));
  v.set("max_indicator", json_number(e.max_indicator));
  v.set("mean_indicator", json_number(e.mean_indicator));
  return v;
}
inline JsonValue iteration(const AdaptiveIteration &it) {
  auto v = JsonValue::object();
  v.set("index", json_number(it.index));
  v.set("record", to_json(it.record));
  auto solver = JsonValue::object();
  solver.set("iterations", json_number(it.solver_iterations));
  solver.set("initial_guess",
             json_string(initial_guess_name(it.initial_guess)));
  v.set("solver", std::move(solver));
  if (const auto &t = it.transfer) {
    auto transfer = JsonValue::object();
    transfer.set("source_nodes", json_number(t->source_nodes));
    transfer.set("target_nodes", json_number(t->target_nodes));
    transfer.set("extrapolated_nodes", json_number(t->extrapolated_nodes));
    transfer.set("max_outside_distance", json_number(t->max_outside_distance));
    v.set("transfer", std::move(transfer));
  } else
    v.set("transfer", JsonValue::null());
  v.set("estimate", estimate(it.estimate));
  if (const auto &m = it.marking) {
    auto marking = JsonValue::object();
    marking.set("strategy", json_string(m->strategy));
    marking.set("descriptor", record_json_detail::descriptor(m->descriptor));
    marking.set("marked", json_number(m->marked));
    marking.set("elements", json_number(m->elements));
    v.set("marking", std::move(marking));
  } else
    v.set("marking", JsonValue::null());
  v.set("refinement", it.refinement
                          ? record_json_detail::mesh_report(*it.refinement)
                          : JsonValue::null());
  v.set("decisions", record_json_detail::decisions(it.decisions));
  return v;
}
} // namespace adaptivity_json_detail

inline JsonValue to_json(const AdaptiveHistory &h) {
  using namespace adaptivity_json_detail;
  auto v = JsonValue::object();
  v.set("schema", json_string(adaptive_history_schema));
  v.set("schema_version", json_number(adaptive_history_schema_version));
  auto stop = JsonValue::object();
  stop.set("reason", json_string(adaptive_stop_name(h.stop)));
  stop.set("message", json_string(h.stop_message));
  v.set("stop", std::move(stop));
  auto iterations = JsonValue::array();
  for (const auto &it : h.iterations)
    iterations.push(iteration(it));
  v.set("iterations", std::move(iterations));
  auto final_state = JsonValue::object();
  final_state.set("mesh_fingerprint",
                  json_string(discretization::mesh_fingerprint(h.mesh.mesh)));
  final_state.set("nodes", json_number(h.mesh.mesh.nodes.size()));
  final_state.set("elements", json_number(h.mesh.mesh.elements.size()));
  final_state.set("estimate", json_number(h.estimate.global));
  v.set("final", std::move(final_state));
  return v;
}
inline std::ostream &write_json(std::ostream &out, const AdaptiveHistory &h) {
  return write_json(out, to_json(h));
}

// Parse an adaptive history document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not an adaptive history",
// "Unsupported adaptive history schema version: N").
inline JsonValue read_adaptive_history_json(std::istream &in) {
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != adaptive_history_schema)
    throw std::invalid_argument("Not an adaptive history");
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text != std::to_string(adaptive_history_schema_version))
    throw std::invalid_argument(
        "Unsupported adaptive history schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}
} // namespace pde2d::fem
