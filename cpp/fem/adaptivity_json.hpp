#pragma once
// Versioned JSON of an adaptive loop (adaptivity.hpp), with the conventions
// of run records (record_json.hpp): stable keys, classic-locale numbers with
// max_digits10 digits, null for absent values.
//
//   write_json(out, history);               // "pde2d.adaptive_history"
//   auto doc = read_adaptive_history_json(in);  // checks schema + version
//   diff_records(to_json(a), to_json(b));    // leaf by leaf
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
inline JsonValue transfer_options(const TransferOptions &o) {
  auto v = JsonValue::object();
  v.set("max_outside_distance", json_number(o.max_outside_distance));
  v.set("execution", record_json_detail::execution(o.execution));
  return v;
}
inline JsonValue optional_text(const std::optional<std::string> &text) {
  return text ? json_string(*text) : JsonValue::null();
}
inline JsonValue settings(const AdaptiveSettings &s) {
  auto v = JsonValue::object();
  auto estimator = JsonValue::object();
  estimator.set("name", json_string(s.estimator));
  estimator.set("descriptor",
                record_json_detail::descriptor(s.estimator_descriptor));
  estimator.set("default", json_bool(s.default_estimator));
  v.set("estimator", std::move(estimator));
  auto marker = JsonValue::object();
  marker.set("name", json_string(s.marker));
  marker.set("descriptor", record_json_detail::descriptor(s.marker_descriptor));
  marker.set("default", json_bool(s.default_marker));
  v.set("marker", std::move(marker));
  v.set("max_iterations", json_number(s.max_iterations));
  v.set("tolerance", json_number(s.tolerance));
  v.set("refinement", s.refinement
                          ? record_json_detail::mesh_options(*s.refinement)
                          : JsonValue::null());
  v.set("warm_start", json_bool(s.warm_start));
  v.set("transfer", transfer_options(s.transfer));
  v.set("time", json_number(s.time));
  v.set("accept_unsatisfied_refinement",
        json_bool(s.accept_unsatisfied_refinement));
  v.set("solve_planner", optional_text(s.solve_planner));
  v.set("mesh_planner", optional_text(s.mesh_planner));
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
  v.set("refinement_options",
        it.refinement_options
            ? record_json_detail::mesh_options(*it.refinement_options)
            : JsonValue::null());
  v.set("refinement_run", it.refinement_run
                              ? discretization::to_json(*it.refinement_run)
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
  v.set("settings", settings(h.settings));
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
inline constexpr const char *evolution_adaptive_history_schema =
    "pde2d.evolution_adaptive_history";
inline constexpr int evolution_adaptive_history_schema_version = 1;

namespace adaptivity_json_detail {
inline JsonValue transfer(const TransferRecord &t) {
  auto v = JsonValue::object();
  v.set("source_nodes", json_number(t.source_nodes));
  v.set("target_nodes", json_number(t.target_nodes));
  v.set("extrapolated_nodes", json_number(t.extrapolated_nodes));
  v.set("max_outside_distance", json_number(t.max_outside_distance));
  return v;
}
inline JsonValue window(const EvolutionWindow &w) {
  auto v = JsonValue::object();
  v.set("index", json_number(w.index));
  v.set("start", json_number(w.start));
  v.set("end", json_number(w.end));
  v.set("record", to_json(w.record));
  if (const auto &r = w.restart) {
    auto restart = JsonValue::object();
    restart.set("values", transfer(r->values));
    restart.set("velocity",
                r->velocity ? transfer(*r->velocity) : JsonValue::null());
    restart.set("dirichlet_nodes", json_number(r->dirichlet_nodes));
    restart.set("max_dirichlet_correction",
                json_number(r->max_dirichlet_correction));
    restart.set("max_velocity_correction",
                json_number(r->max_velocity_correction));
    v.set("restart", std::move(restart));
  } else
    v.set("restart", JsonValue::null());
  v.set("estimate", estimate(w.estimate));
  if (const auto &m = w.marking) {
    auto marking = JsonValue::object();
    marking.set("strategy", json_string(m->strategy));
    marking.set("descriptor", record_json_detail::descriptor(m->descriptor));
    marking.set("marked", json_number(m->marked));
    marking.set("elements", json_number(m->elements));
    v.set("marking", std::move(marking));
  } else
    v.set("marking", JsonValue::null());
  v.set("refinement_options",
        w.refinement_options
            ? record_json_detail::mesh_options(*w.refinement_options)
            : JsonValue::null());
  v.set("refinement", w.refinement
                          ? record_json_detail::mesh_report(*w.refinement)
                          : JsonValue::null());
  v.set("refinement_run", w.refinement_run
                              ? discretization::to_json(*w.refinement_run)
                              : JsonValue::null());
  v.set("refinement_accepted", json_bool(w.refinement_accepted));
  v.set("decisions", record_json_detail::decisions(w.decisions));
  return v;
}
inline JsonValue settings(const EvolutionAdaptSettings &s) {
  auto v = JsonValue::object();
  v.set("windows", json_number(s.windows));
  auto estimator = JsonValue::object();
  estimator.set("name", json_string(s.estimator));
  estimator.set("descriptor",
                record_json_detail::descriptor(s.estimator_descriptor));
  estimator.set("default", json_bool(s.default_estimator));
  v.set("estimator", std::move(estimator));
  auto marker = JsonValue::object();
  marker.set("name", json_string(s.marker));
  marker.set("descriptor", record_json_detail::descriptor(s.marker_descriptor));
  marker.set("default", json_bool(s.default_marker));
  v.set("marker", std::move(marker));
  v.set("tolerance", json_number(s.tolerance));
  v.set("refinement", s.refinement
                          ? record_json_detail::mesh_options(*s.refinement)
                          : JsonValue::null());
  v.set("transfer", transfer_options(s.transfer));
  v.set("accept_unsatisfied_refinement",
        json_bool(s.accept_unsatisfied_refinement));
  v.set("start", json_number(s.start));
  v.set("end", json_number(s.end));
  v.set("step", json_number(s.step));
  return v;
}
} // namespace adaptivity_json_detail

// Schema evolution_adaptive_history_schema (version 1): settings, one
// object per window (embedding its run record) and the final state.
inline JsonValue to_json(const EvolutionAdaptiveHistory &h) {
  using namespace adaptivity_json_detail;
  auto v = JsonValue::object();
  v.set("schema", json_string(evolution_adaptive_history_schema));
  v.set("schema_version",
        json_number(evolution_adaptive_history_schema_version));
  v.set("settings", settings(h.settings));
  auto windows = JsonValue::array();
  for (const auto &w : h.windows)
    windows.push(window(w));
  v.set("windows", std::move(windows));
  auto final_state = JsonValue::object();
  final_state.set("mesh_fingerprint",
                  json_string(discretization::mesh_fingerprint(h.mesh.mesh)));
  final_state.set("nodes", json_number(h.mesh.mesh.nodes.size()));
  final_state.set("elements", json_number(h.mesh.mesh.elements.size()));
  final_state.set("time", h.evolution.frames.empty()
                              ? JsonValue::null()
                              : json_number(h.evolution.frames.back().time));
  final_state.set("estimate", json_number(h.estimate.global));
  v.set("final", std::move(final_state));
  return v;
}
inline std::ostream &write_json(std::ostream &out,
                                const EvolutionAdaptiveHistory &h) {
  return write_json(out, to_json(h));
}
// Parse a transient adaptive history and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not an evolution adaptive
// history", "Unsupported evolution adaptive history schema version: N").
inline JsonValue read_evolution_adaptive_history_json(std::istream &in) {
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != evolution_adaptive_history_schema)
    throw std::invalid_argument("Not an evolution adaptive history");
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text !=
          std::to_string(evolution_adaptive_history_schema_version))
    throw std::invalid_argument(
        "Unsupported evolution adaptive history schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}
} // namespace pde2d::fem
