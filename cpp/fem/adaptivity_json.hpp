#pragma once
// Versioned JSON of an adaptive loop (adaptivity.hpp), with the conventions
// of run records (record_json.hpp): stable keys, classic-locale numbers with
// max_digits10 digits, null for absent values.
//
//   write_json(out, history);               // "pde2d.adaptive_history"
//   auto doc = read_adaptive_history_json(in);  // checks schema + version
//   diff_records(to_json(a), to_json(b));    // leaf by leaf
//
// One schema for both drivers ("kind": "stationary" or "transient"). Every
// step embeds the full run record of its solve or window under "record"
// (schema "pde2d.run_record"). Keys are documented in docs/fem/README.md
// ("Adaptive loop"); adding a key keeps the schema version, renaming or
// removing one increments it. Version 2 merged the stationary and transient
// histories (version 1 had "iterations" and a separate
// "pde2d.evolution_adaptive_history"). Indicators and nodal values are not
// written.
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_record_json.hpp"
#include "fem/adaptivity.hpp"
#include "fem/json.hpp"
#include "fem/record_json.hpp"
#include <istream>
#include <ostream>
#include <string>

namespace pde2d::fem {
inline constexpr const char *adaptive_history_schema = "pde2d.adaptive_history";
inline constexpr int adaptive_history_schema_version = 2;

namespace adaptivity_json_detail {
inline JsonValue strategy(const std::string &name,
                          const std::optional<pde2d::StrategyDescriptor> &d) {
  auto v = JsonValue::object();
  v.set("name", json_string(name));
  v.set("descriptor", record_json_detail::descriptor(d));
  return v;
}
inline JsonValue optional_number(const std::optional<double> &x) {
  return x ? json_number(*x) : JsonValue::null();
}
inline JsonValue estimate(const EstimateRecord &e) {
  auto v = JsonValue::object();
  v.set("estimator", json_string(e.estimator));
  v.set("descriptor", record_json_detail::descriptor(e.descriptor));
  v.set("global", json_number(e.global));
  v.set("elements", json_number(e.elements));
  v.set("max_indicator", json_number(e.max_indicator));
  v.set("mean_indicator", json_number(e.mean_indicator));
  return v;
}
inline JsonValue transfer(const std::optional<TransferRecord> &t) {
  if (!t)
    return JsonValue::null();
  auto v = JsonValue::object();
  v.set("source_nodes", json_number(t->source_nodes));
  v.set("target_nodes", json_number(t->target_nodes));
  v.set("extrapolated_nodes", json_number(t->extrapolated_nodes));
  v.set("max_outside_distance", json_number(t->max_outside_distance));
  v.set("dirichlet_nodes", json_number(t->dirichlet_nodes));
  v.set("max_dirichlet_correction", json_number(t->max_dirichlet_correction));
  return v;
}
template <class Planner>
JsonValue planner(const std::shared_ptr<const Planner> &p) {
  return p ? strategy(p->name(), p->descriptor()) : JsonValue::null();
}
inline JsonValue settings(const AdaptiveHistory &h) {
  const auto &o = h.options;
  const bool transient = h.evolution.has_value();
  auto v = JsonValue::object();
  v.set("estimator", strategy(o.estimator->name(), o.estimator->descriptor()));
  v.set("marker", strategy(o.marker->name(), o.marker->descriptor()));
  v.set("adaptation",
        strategy(o.adaptation->name(), o.adaptation->descriptor()));
  v.set("solve_planner", planner(o.planners.solve));
  v.set("mesh_planner", planner(o.planners.mesh));
  v.set("tolerance", json_number(o.tolerance));
  v.set("refinement", o.refinement
                          ? record_json_detail::mesh_options(*o.refinement)
                          : JsonValue::null());
  auto transfer_options = JsonValue::object();
  transfer_options.set("max_outside_distance",
                       json_number(o.transfer.max_outside_distance));
  transfer_options.set("execution",
                       record_json_detail::execution(o.transfer.execution));
  v.set("transfer", std::move(transfer_options));
  v.set("accept_unsatisfied_refinement",
        json_bool(o.accept_unsatisfied_refinement));
  if (!transient) {
    v.set("max_iterations", json_number(o.max_iterations));
    v.set("warm_start", json_bool(o.warm_start));
    v.set("time", json_number(o.time));
  } else {
    // The whole interval and step, from the first and last window.
    v.set("windows", json_number(o.windows));
    const auto &first = h.steps.front(), &last = h.steps.back();
    v.set("start", optional_number(first.window_start));
    v.set("end", json_number(last.time));
    const auto &time = first.record.plan.time;
    v.set("step", time ? json_number(time->step) : JsonValue::null());
  }
  return v;
}
inline JsonValue step(const AdaptiveStep &s) {
  auto v = JsonValue::object();
  v.set("index", json_number(s.index));
  v.set("window_start", optional_number(s.window_start));
  v.set("time", json_number(s.time));
  v.set("record", to_json(s.record));
  v.set("transfer", transfer(s.transfer));
  v.set("velocity_transfer", transfer(s.velocity_transfer));
  v.set("estimate", estimate(s.estimate));
  if (const auto &m = s.marking) {
    auto marking = JsonValue::object();
    marking.set("strategy", json_string(m->strategy));
    marking.set("descriptor", record_json_detail::descriptor(m->descriptor));
    marking.set("marked", json_number(m->marked));
    marking.set("elements", json_number(m->elements));
    v.set("marking", std::move(marking));
  } else
    v.set("marking", JsonValue::null());
  if (const auto &a = s.adaptation) {
    auto adaptation = JsonValue::object();
    adaptation.set("strategy", json_string(a->strategy));
    adaptation.set("descriptor", record_json_detail::descriptor(a->descriptor));
    adaptation.set("report", record_json_detail::mesh_report(a->report));
    adaptation.set("run", discretization::to_json(a->run));
    adaptation.set("accepted", json_bool(a->accepted));
    v.set("adaptation", std::move(adaptation));
  } else
    v.set("adaptation", JsonValue::null());
  v.set("decisions", record_json_detail::decisions(s.decisions));
  return v;
}
} // namespace adaptivity_json_detail

inline JsonValue to_json(const AdaptiveHistory &h) {
  using namespace adaptivity_json_detail;
  auto v = JsonValue::object();
  v.set("schema", json_string(adaptive_history_schema));
  v.set("schema_version", json_number(adaptive_history_schema_version));
  v.set("kind", json_string(h.evolution ? "transient" : "stationary"));
  auto stop = JsonValue::object();
  stop.set("reason", json_string(adaptive_stop_name(h.stop)));
  stop.set("message", json_string(h.stop_message));
  v.set("stop", std::move(stop));
  v.set("settings", settings(h));
  auto steps = JsonValue::array();
  for (const auto &s : h.steps)
    steps.push(step(s));
  v.set("steps", std::move(steps));
  auto final_state = JsonValue::object();
  final_state.set("mesh_fingerprint",
                  json_string(discretization::mesh_fingerprint(h.mesh.mesh)));
  final_state.set("nodes", json_number(h.mesh.mesh.nodes.size()));
  final_state.set("elements", json_number(h.mesh.mesh.elements.size()));
  final_state.set("time", h.evolution && !h.evolution->frames.empty()
                              ? json_number(h.evolution->frames.back().time)
                              : json_number(h.options.time));
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
