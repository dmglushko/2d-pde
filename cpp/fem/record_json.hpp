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
// changing a value's meaning, increments it. Numbers use the classic locale
// and max_digits10 digits (cpp/json.hpp), so a parsed record has exactly
// the doubles of the run. Factories are not serialized: a user backend or
// integrator is written by name with "user_supplied": true.
#include "../json.hpp"
#include "discretization/mesh_record_json.hpp"
#include "fem/record.hpp"
#include "fem/time_integration.hpp"
#include <istream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
inline constexpr const char *run_record_schema = "pde2d.run_record";
inline constexpr int run_record_schema_version = 1;

namespace record_json_detail {
inline const char *backend_name(pde2d::ExecutionBackend backend) {
  return backend == pde2d::ExecutionBackend::OpenMP ? "OpenMP" : "Serial";
}
inline JsonValue strategy(const StrategyRecord &s) {
  auto v = JsonValue::object();
  v.set("name", json_string(s.name));
  v.set("user_supplied", json_bool(s.user_supplied));
  return v;
}
inline JsonValue field(const FieldRecord &f) {
  auto v = JsonValue::object();
  v.set("kind", json_string(f.kind));
  v.set("fingerprint", json_string(f.fingerprint));
  return v;
}
inline JsonValue assembly(const AssemblyOptions &a) {
  auto execution = JsonValue::object();
  execution.set("backend", json_string(backend_name(a.execution.backend)));
  execution.set("threads", json_number(a.execution.threads));
  execution.set("min_parallel_size",
                json_number(a.execution.min_parallel_size));
  auto v = JsonValue::object();
  v.set("quadrature_degree", json_number(a.quadrature_degree));
  v.set("boundary_tolerance", json_number(a.boundary_tolerance));
  v.set("execution", std::move(execution));
  return v;
}
inline JsonValue solver(const SolverOptions &o, const StrategyRecord &s) {
  const char *method = linear_solver_method_name(o.method);
  auto v = JsonValue::object();
  v.set("name", json_string(s.name));
  v.set("user_supplied", json_bool(s.user_supplied));
  v.set("method", json_string(method ? method : "Unknown"));
  v.set("relative_tolerance", json_number(o.relative_tolerance));
  v.set("absolute_tolerance", json_number(o.absolute_tolerance));
  v.set("max_iterations", json_number(o.max_iterations));
  v.set("restart", json_number(o.restart));
  v.set("validation", json_string(solver_validation_name(o.validation)));
  v.set("preconditioner", json_string(preconditioner_name(o.preconditioner)));
  return v;
}
inline JsonValue time(const TimeRecord &t) {
  auto v = JsonValue::object();
  v.set("method", json_string(time_method_name(t.method)));
  v.set("integrator", strategy(t.integrator));
  v.set("start", json_number(t.start));
  v.set("end", json_number(t.end));
  v.set("step", json_number(t.step));
  v.set("frame_stride", json_number(t.frame_stride));
  v.set("store_reports", json_bool(t.store_reports));
  v.set("observer", json_bool(t.observer));
  return v;
}
inline JsonValue stationary(const StationaryRecord &s) {
  auto outcome = JsonValue::object();
  outcome.set("iterations", json_number(s.iterations));
  outcome.set("converged", json_bool(s.converged));
  outcome.set("residual_norm", json_number(s.residual_norm));
  outcome.set("relative_residual", json_number(s.relative_residual));
  auto v = JsonValue::object();
  v.set("time", json_number(s.time));
  v.set("initial_guess", json_string(initial_guess_name(s.initial_guess)));
  v.set("guess", s.guess ? field(*s.guess) : JsonValue::null());
  v.set("outcome", std::move(outcome));
  return v;
}
} // namespace record_json_detail

// The JSON document of a record (schema run_record_schema, version
// run_record_schema_version).
inline JsonValue to_json(const RunRecord &r) {
  using namespace record_json_detail;
  auto v = JsonValue::object();
  v.set("schema", json_string(run_record_schema));
  v.set("schema_version", json_number(run_record_schema_version));
  v.set("kind", json_string(r.time ? "evolution" : "stationary"));
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
  mesh.set("fingerprint", json_string(r.mesh_fingerprint));
  mesh.set("run", r.mesh_run ? discretization::to_json(*r.mesh_run)
                             : JsonValue::null());
  v.set("mesh", std::move(mesh));
  auto workload = JsonValue::object();
  workload.set("nodes", json_number(r.workload.nodes));
  workload.set("elements", json_number(r.workload.elements));
  workload.set("boundary_edges", json_number(r.workload.boundary_edges));
  v.set("workload", std::move(workload));
  if (r.initial) {
    auto initial = JsonValue::object();
    initial.set("value", field(r.initial->value));
    initial.set("velocity", r.initial->velocity ? field(*r.initial->velocity)
                                                : JsonValue::null());
    v.set("initial", std::move(initial));
  } else
    v.set("initial", JsonValue::null());
  auto options = JsonValue::object();
  options.set("assembly", assembly(r.assembly));
  options.set("solver", solver(r.solver, r.linear_solver));
  options.set("time", r.time ? time(*r.time) : JsonValue::null());
  v.set("options", std::move(options));
  auto execution = JsonValue::object();
  execution.set("openmp_available", json_bool(r.openmp_available));
  v.set("execution", std::move(execution));
  v.set("stationary",
        r.stationary ? stationary(*r.stationary) : JsonValue::null());
  const auto &w = r.work;
  auto work = JsonValue::object();
  work.set("assembly_calls", json_number(w.assembly_calls));
  work.set("linear_solves", json_number(w.linear_solves));
  work.set("solver_preparations", json_number(w.solver_preparations));
  work.set("solver_reuses", json_number(w.solver_reuses));
  work.set("time_steps", json_number(w.time_steps));
  work.set("solver_iterations", json_number(w.solver_iterations));
  v.set("work", std::move(work));
  auto timings = JsonValue::object();
  timings.set("total_seconds", json_number(r.timings.total_seconds));
  timings.set("assembly_seconds", json_number(r.timings.assembly_seconds));
  timings.set("solver_seconds", json_number(r.timings.solver_seconds));
  v.set("timings", std::move(timings));
  return v;
}

inline std::ostream &write_json(std::ostream &out, const RunRecord &record) {
  return pde2d::write_json(out, to_json(record));
}

// Parse a record document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a run record",
// "Unsupported run record schema version: N").
inline JsonValue read_run_record_json(std::istream &in) {
  return json_detail::read_schema(in, run_record_schema,
                                  run_record_schema_version, "run record");
}

// Leaves of the two documents that differ (pde2d::diff_records): by default
// without timings and solver outcomes.
inline std::vector<RecordDifference>
diff_records(const RunRecord &left, const RunRecord &right,
             const RecordDiffOptions &options = {}) {
  return pde2d::diff_records(to_json(left), to_json(right), options);
}
} // namespace pde2d::fem
