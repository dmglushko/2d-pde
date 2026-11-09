#pragma once
// Versioned, machine-readable run records (record.hpp).
//
//   write_json(out, record);                      // schema "pde2d.run_record"
//   JsonValue doc = read_run_record_json(in);     // checks schema + version
//   auto diff = diff_records(record_a, record_b); // field by field
//   for (const auto &d : diff) std::cout << d.path << ": " << d.left
//                                        << " -> " << d.right << '\n';
//
// Keys are stable and documented in docs/fem/README.md ("Run records").
// Adding a key keeps the schema version; renaming or removing one, or
// changing a value's meaning, increments it. Numbers use the classic
// locale and max_digits10 digits (json.hpp), so a parsed record has exactly
// the doubles of the run. Strategy factories are not serialized: the
// "strategies" array names what ran and whether it is replayable.
#include "fem/analysis_types.hpp"
#include "fem/boundary_form.hpp"
#include "fem/json.hpp"
#include "fem/record.hpp"
#include <istream>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
inline constexpr const char *run_record_schema = "pde2d.run_record";
inline constexpr int run_record_schema_version = 1;

namespace record_json_detail {
inline JsonValue optional_number(const std::optional<double> &value) {
  return value ? json_number(*value) : JsonValue::null();
}
inline const char *backend_name(pde2d::ExecutionBackend backend) {
  return backend == pde2d::ExecutionBackend::OpenMP ? "OpenMP" : "Serial";
}
inline JsonValue execution(const pde2d::ExecutionOptions &e) {
  auto v = JsonValue::object();
  v.set("backend", json_string(backend_name(e.backend)));
  v.set("threads", json_number(e.threads));
  v.set("min_parallel_size", json_number(e.min_parallel_size));
  return v;
}
inline JsonValue descriptor(const std::optional<pde2d::StrategyDescriptor> &d) {
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
inline JsonValue decisions(const std::vector<Decision> &list) {
  auto v = JsonValue::array();
  for (const auto &d : list) {
    auto item = JsonValue::object();
    item.set("subject", json_string(decision_subject_name(d.subject)));
    item.set("choice", json_string(d.choice));
    item.set("basis", json_string(d.basis));
    item.set("evidence", json_string(evidence_name(d.evidence)));
    item.set("source", json_string(decision_source_name(d.source)));
    v.push(std::move(item));
  }
  return v;
}
inline JsonValue mesh_options(const discretization::MeshOptions &o) {
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
                 descriptor(o.refinement.size_field_descriptor));
  refinement.set("criterion", json_bool(o.refinement.criterion.has_value()));
  refinement.set("criterion_descriptor",
                 o.refinement.criterion
                     ? descriptor(o.refinement.criterion->descriptor)
                     : JsonValue::null());
  auto v = JsonValue::object();
  v.set("algorithm",
        json_string(discretization::interior_algorithm_name(o.algorithm)));
  v.set("boundary_treatment",
        json_string(
            discretization::boundary_treatment_name(o.boundary_treatment)));
  v.set("preserve_regions", json_bool(o.preserve_regions));
  v.set("boundary", std::move(boundary));
  v.set("refinement", std::move(refinement));
  return v;
}
inline JsonValue mesh_report(const discretization::MeshReport &r) {
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
template <class T, class Name>
JsonValue property(const Property<T> &p, Name &&name) {
  auto v = JsonValue::object();
  v.set("value", name(p.value));
  v.set("evidence", json_string(evidence_name(p.evidence)));
  v.set("reason", json_string(p.reason));
  return v;
}
inline JsonValue analysis(const AnalysisRecord &a) {
  const auto sign = [](Sign s) { return json_string(sign_name(s)); };
  const auto boolean = [](bool b) { return json_bool(b); };
  auto options = JsonValue::object();
  options.set("quadrature_degree", json_number(a.options.quadrature_degree));
  auto times = JsonValue::array();
  for (const double t : a.options.sample_times)
    times.push(json_number(t));
  options.set("sample_times", std::move(times));
  const auto &p = a.properties;
  auto properties = JsonValue::object();
  properties.set("time_order",
                 property(p.time_order, [](int v) { return json_number(v); }));
  properties.set("first_mass", property(p.first_mass, sign));
  properties.set("second_mass", property(p.second_mass, sign));
  properties.set("time_dependent",
                 property(p.time_dependent, [](const WeakFlags &flags) {
                   std::string terms;
                   for (const bool f : flags)
                     terms += f ? '1' : '0';
                   return json_string(terms);
                 }));
  properties.set("symmetric", property(p.symmetric, boolean));
  properties.set("diffusion", property(p.diffusion, [](Definiteness d) {
                   return json_string(definiteness_name(d));
                 }));
  properties.set("reaction", property(p.reaction, sign));
  properties.set("has_dirichlet", property(p.has_dirichlet, boolean));
  properties.set("constant_nullspace", property(p.constant_nullspace, boolean));
  auto regions = JsonValue::array();
  for (const auto &r : p.boundaries) {
    auto region = JsonValue::object();
    region.set("region", json_string(r.region));
    region.set("edges", json_number(r.edges));
    region.set("dirichlet_edges", json_number(r.dirichlet_edges));
    region.set("natural_edges", json_number(r.natural_edges));
    region.set("oblique_edges", json_number(r.oblique_edges));
    region.set("form", property(r.form, [](BoundaryForm f) {
                 return json_string(boundary_form_name(f));
               }));
    region.set("divisor_keeps_sign", property(r.divisor_keeps_sign, boolean));
    region.set("time_dependent", property(r.time_dependent, boolean));
    regions.push(std::move(region));
  }
  properties.set("boundaries", std::move(regions));
  auto v = JsonValue::object();
  v.set("options", std::move(options));
  v.set("properties", std::move(properties));
  return v;
}
inline JsonValue plan(const SolvePlan &plan) {
  const auto &a = plan.assembly;
  auto assembly = JsonValue::object();
  assembly.set("quadrature_degree", json_number(a.quadrature_degree));
  assembly.set("boundary_tolerance", json_number(a.boundary_tolerance));
  assembly.set("execution", execution(a.execution));
  const auto &o = plan.solver;
  SolverOptions builtin;
  builtin.method = o.method;
  auto solver = JsonValue::object();
  solver.set("method", json_string(linear_solver_name(builtin)));
  solver.set("backend",
             o.backend ? json_string(o.backend->name()) : JsonValue::null());
  solver.set("relative_tolerance", json_number(o.relative_tolerance));
  solver.set("absolute_tolerance", json_number(o.absolute_tolerance));
  solver.set("max_iterations", json_number(o.max_iterations));
  solver.set("restart", json_number(o.restart));
  solver.set("validation", json_string(solver_validation_name(o.validation)));
  solver.set("preconditioner",
             json_string(preconditioner_name(o.preconditioner)));
  auto v = JsonValue::object();
  v.set("assembly", std::move(assembly));
  v.set("solver", std::move(solver));
  if (const auto &t = plan.time) {
    auto time = JsonValue::object();
    time.set("method", json_string(time_method_name(t->method)));
    time.set("start", json_number(t->start));
    time.set("end", json_number(t->end));
    time.set("step", json_number(t->step));
    time.set("integrator", json_bool(static_cast<bool>(t->integrator)));
    time.set("step_controller",
             json_bool(static_cast<bool>(t->step_controller)));
    time.set("max_consecutive_rejections",
             json_number(t->max_consecutive_rejections));
    v.set("time", std::move(time));
  } else
    v.set("time", JsonValue::null());
  v.set("decisions", decisions(plan.decisions));
  if (plan.mesh) {
    auto mesh = JsonValue::object();
    mesh.set("options", mesh_options(plan.mesh->options));
    mesh.set("decisions", decisions(plan.mesh->decisions));
    v.set("mesh", std::move(mesh));
  } else
    v.set("mesh", JsonValue::null());
  v.set("analysis",
        plan.analysis ? analysis(*plan.analysis) : JsonValue::null());
  return v;
}
inline JsonValue field(const FieldRecord &f) {
  auto v = JsonValue::object();
  v.set("kind", json_string(f.kind));
  v.set("fingerprint", json_string(f.fingerprint));
  return v;
}

// Flattened leaves: "plan.solver.method" -> "\"SparseLU\"".
inline void flatten(const JsonValue &v, const std::string &path,
                    std::vector<std::pair<std::string, std::string>> &out) {
  switch (v.kind) {
  case JsonValue::Kind::Object:
    if (v.members.empty())
      out.emplace_back(path, "{}");
    for (const auto &[key, member] : v.members)
      flatten(member, path.empty() ? key : path + '.' + key, out);
    return;
  case JsonValue::Kind::Array:
    if (v.items.empty())
      out.emplace_back(path, "[]");
    for (std::size_t k = 0; k < v.items.size(); ++k)
      flatten(v.items[k], path + '[' + std::to_string(k) + ']', out);
    return;
  default: {
    auto text = json_text(v);
    text.pop_back(); // the newline json_text appends
    out.emplace_back(path, std::move(text));
  }
  }
}
} // namespace record_json_detail

// The JSON document of a record (schema run_record_schema, version
// run_record_schema_version).
inline JsonValue to_json(const RunRecord &r) {
  using namespace record_json_detail;
  auto v = JsonValue::object();
  v.set("schema", json_string(run_record_schema));
  v.set("schema_version", json_number(run_record_schema_version));
  auto versions = JsonValue::object();
  versions.set("pde2d", json_string(r.versions.library.version));
  versions.set("git_sha", json_string(r.versions.library.git_sha));
  versions.set("eigen", json_string(r.versions.eigen));
  versions.set("symengine", json_string(r.versions.symengine));
  v.set("versions", std::move(versions));
  v.set("replayable", json_bool(r.replayable()));
  auto problem = JsonValue::object();
  problem.set("fingerprint", json_string(r.problem_fingerprint));
  v.set("problem", std::move(problem));
  auto mesh = JsonValue::object();
  mesh.set("fingerprint", json_string(r.mesh.fingerprint));
  if (const auto &p = r.mesh.provenance) {
    auto provenance = JsonValue::object();
    provenance.set("options", mesh_options(p->options));
    provenance.set("report", mesh_report(p->report));
    provenance.set("boundary_context", json_bool(p->boundary_context));
    mesh.set("provenance", std::move(provenance));
  } else
    mesh.set("provenance", JsonValue::null());
  v.set("mesh", std::move(mesh));
  auto workload = JsonValue::object();
  workload.set("nodes", json_number(r.workload.nodes));
  workload.set("elements", json_number(r.workload.elements));
  workload.set("boundary_edges", json_number(r.workload.boundary_edges));
  v.set("workload", std::move(workload));
  v.set("stationary_time", optional_number(r.stationary_time));
  if (r.initial) {
    auto initial = JsonValue::object();
    initial.set("value", field(r.initial->value));
    initial.set("velocity", r.initial->velocity ? field(*r.initial->velocity)
                                                : JsonValue::null());
    v.set("initial", std::move(initial));
  } else
    v.set("initial", JsonValue::null());
  v.set("plan", plan(r.plan));
  auto strategies = JsonValue::array();
  for (const auto &s : r.strategies) {
    auto item = JsonValue::object();
    item.set("subject", json_string(decision_subject_name(s.subject)));
    item.set("name", json_string(s.name));
    item.set("replay", json_string(strategy_replay_name(s.replay)));
    item.set("descriptor", descriptor(s.descriptor));
    strategies.push(std::move(item));
  }
  v.set("strategies", std::move(strategies));
  auto exec = JsonValue::object();
  exec.set("selected", execution(r.execution.selected));
  exec.set("openmp_available", json_bool(r.execution.openmp_available));
  v.set("execution", std::move(exec));
  const auto &w = r.work;
  auto work = JsonValue::object();
  work.set("assembly_calls", json_number(w.assembly_calls));
  work.set("linear_solves", json_number(w.linear_solves));
  work.set("solver_preparations", json_number(w.solver_preparations));
  work.set("solver_reuses", json_number(w.solver_reuses));
  work.set("time_steps", json_number(w.time_steps));
  work.set("rejected_steps", json_number(w.rejected_steps));
  v.set("work", std::move(work));
  auto evidence = JsonValue::object();
  evidence.set("proven", json_number(r.evidence.proven));
  evidence.set("sampled", json_number(r.evidence.sampled));
  evidence.set("unknown", json_number(r.evidence.unknown));
  evidence.set("weakest", json_string(evidence_name(r.evidence.weakest)));
  v.set("evidence", std::move(evidence));
  auto timings = JsonValue::object();
  timings.set("total_seconds", json_number(r.timings.total_seconds));
  v.set("timings", std::move(timings));
  return v;
}

inline std::ostream &write_json(std::ostream &out, const RunRecord &record) {
  return write_json(out, to_json(record));
}

// Parse a record document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a run record",
// "Unsupported run record schema version: N").
inline JsonValue read_run_record_json(std::istream &in) {
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != run_record_schema)
    throw std::invalid_argument("Not a run record");
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text != std::to_string(run_record_schema_version))
    throw std::invalid_argument(
        "Unsupported run record schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}

// One differing leaf: its dotted path ("plan.decisions[2].choice") and the
// JSON text on each side ("<absent>" when one side lacks the path).
struct RecordDifference {
  std::string path, left, right;
  bool operator==(const RecordDifference &) const = default;
};
struct RecordDiffOptions {
  // Skip "timings.*", the only field that differs between identical runs
  // (also nested, as in the run records of an adaptive history:
  // "iterations[0].record.timings.total_seconds").
  bool ignore_timings = true;
};
// Leaves that differ, in the order of `left`, then leaves only `right` has.
inline std::vector<RecordDifference>
diff_records(const JsonValue &left, const JsonValue &right,
             const RecordDiffOptions &options = {}) {
  std::vector<std::pair<std::string, std::string>> a, b;
  record_json_detail::flatten(left, "", a);
  record_json_detail::flatten(right, "", b);
  const auto ignored = [&](const std::string &path) {
    return options.ignore_timings &&
           (path == "timings" || path.starts_with("timings.") ||
            path.ends_with(".timings") ||
            path.find(".timings.") != std::string::npos);
  };
  std::map<std::string, std::string> rhs(b.begin(), b.end());
  std::vector<RecordDifference> differences;
  for (const auto &[path, value] : a) {
    if (ignored(path))
      continue;
    const auto found = rhs.find(path);
    if (found == rhs.end())
      differences.push_back({path, value, "<absent>"});
    else if (found->second != value)
      differences.push_back({path, value, found->second});
  }
  std::map<std::string, std::string> lhs(a.begin(), a.end());
  for (const auto &[path, value] : b)
    if (!ignored(path) && !lhs.contains(path))
      differences.push_back({path, "<absent>", value});
  return differences;
}
inline std::vector<RecordDifference>
diff_records(const RunRecord &left, const RunRecord &right,
             const RecordDiffOptions &options = {}) {
  return diff_records(to_json(left), to_json(right), options);
}
} // namespace pde2d::fem
