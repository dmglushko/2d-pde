#pragma once
// Time evolution driver.
//
//   Problem          spatial equation + boundary conditions (problem.hpp)
//   InitialConditions u (and u_t for second-order problems) at options.start,
//                     each an Expr or one value per mesh node (Field)
//   TimeOptions       scheme (built-in TimeMethod or a user TimeIntegrator),
//                     interval/step, assembly/solver options, output
//
//   Evolution e = evolve(mesh, problem, {u0, v0}, options);
//
// evolve() owns the caching (one PreparedAssembly, reuse of a factorization
// while the constrained matrix is unchanged), output frames, the optional
// observer, work/timing accounting and the checks of what integrators
// return. The numerical recurrence of each scheme lives in
// time_integration.hpp and the step sequence in time_grid.hpp.
// Evolution::record (RunRecord, record.hpp) holds the options and time
// settings that ran, the integrator's name, workload, work, input
// fingerprints (mesh, problem, initial fields) and versions.
#include "discretization/mesh_fingerprint.hpp"
#include "fem/assembly.hpp"
#include "fem/constraints.hpp"
#include "fem/fingerprint.hpp"
#include "fem/interpolation.hpp"
#include "fem/prepared_linear_solver.hpp"
#include "fem/record.hpp"
#include "fem/solver.hpp"
#include "fem/time_grid.hpp"
#include "fem/time_integration.hpp"
#include "fem/timing.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
struct InitialConditions {
  // u(x, y, start): an expression, or one value per mesh node (for example
  // the last frame of an earlier evolution).
  Field value = Expr(0);
  // u_t(x, y, start). Required by second-order schemes (Newmark). First-order
  // schemes only report it in the initial frame (0 when omitted).
  std::optional<Field> velocity{};
};
struct Frame {
  double time;
  Vector values;
  Vector velocity;
};
struct TimeOptions {
  TimeMethod method = TimeMethod::BackwardEuler;
  double start = 0, end = 1, step = .01;
  // A user scheme. When set, evolve() calls it once and ignores `method`.
  TimeIntegratorFactory integrator{};
  AssemblyOptions assembly;
  SolverOptions solver;
  // Retain frames 0, stride, 2*stride, ... and the final frame.
  std::size_t frame_stride = 1;
  bool store_reports = true;
  // Called for every time stamp, retained or not.
  std::function<void(const Frame &)> observer{};
};
struct Evolution {
  std::vector<Frame> frames;
  std::vector<SolverReport> reports;
  EvolutionTimings timings;
  // What ran (record.hpp): options, time settings and the integrator's name
  // (a user integrator's name() when TimeOptions::integrator is set),
  // workload, work, fingerprints and versions. record.assembly.execution is
  // the selected policy, not an observation of actual OpenMP team sizes.
  RunRecord record;
};

namespace evolution_detail {
// Constrain, then solve with the cached factorization when the constrained
// matrix is coefficient-for-coefficient unchanged.
inline Vector solve_step(const SparseMatrix &a, const Vector &b,
                         const std::vector<DirichletValue> &data,
                         const SolverOptions &options, Evolution &result,
                         std::unique_ptr<PreparedLinearSolver> &prepared,
                         bool store_report) {
  auto constrained = [&] {
    timing::Accumulate _(result.timings.constraints_seconds);
    return constrain(a, b, data);
  }();
  auto solution = [&] {
    timing::Accumulate _(result.timings.solver_seconds);
    if (!prepared || !prepared->matches(constrained.first, options)) {
      prepared =
          std::make_unique<PreparedLinearSolver>(constrained.first, options);
      result.timings.solver += prepared->preparation_timings();
    } else
      ++result.timings.solver_reuses;
    return prepared->solve(constrained.second);
  }();
  result.timings.solver += solution.report.timings;
  ++result.timings.linear_solves;
  result.record.work.solver_iterations +=
      record_detail::iterations(solution.report);
  if (!solution.report.converged)
    throw std::runtime_error("Evolution solve: " + solution.report.message);
  if (store_report)
    result.reports.push_back(solution.report);
  return solution.values;
}

// What an integrator returned must be usable before the driver indexes it
// (Dirichlet enforcement) or hands it to the observer.
inline void check_state(const IntegratorState &state, Eigen::Index nodes,
                        const char *phase) {
  if (state.u.size() != nodes || state.v.size() != nodes)
    throw std::runtime_error(std::string("Time integrator ") + phase +
                             " returned a state of the wrong size");
  if (!state.u.allFinite() || !state.v.allFinite())
    throw std::runtime_error(std::string("Nonfinite evolution output from "
                                         "time integrator ") +
                             phase);
}

// Complete result.record once the evolution has finished (the solve
// iterations were summed by solve_step).
inline void record(Evolution &result, const discretization::TriangleMesh &mesh,
                   const Problem &problem, const InitialConditions &initial,
                   const TimeOptions &options,
                   const TimeIntegrator &integrator) {
  auto &r = result.record;
  const auto &t = result.timings;
  r.assembly = options.assembly;
  r.solver = options.solver;
  r.linear_solver = record_detail::linear_solver(options.solver);
  r.time =
      TimeRecord{options.method,
                 {integrator.name(), static_cast<bool>(options.integrator)},
                 options.start,
                 options.end,
                 options.step,
                 options.frame_stride,
                 options.store_reports,
                 static_cast<bool>(options.observer)};
  r.openmp_available = pde2d::openmp_available();
  r.workload = record_detail::workload(mesh);
  r.work.assembly_calls = t.assembly_calls;
  r.work.linear_solves = t.linear_solves;
  r.work.solver_preparations = t.solver.preparations;
  r.work.solver_reuses = t.solver_reuses;
  r.work.time_steps = t.time_steps;
  const auto field = [](const Field &f) {
    return FieldRecord{field_kind(f), field_fingerprint(f)};
  };
  r.initial = InitialRecord{field(initial.value), std::nullopt};
  if (initial.velocity)
    r.initial->velocity = field(*initial.velocity);
  r.mesh_fingerprint = discretization::mesh_fingerprint(mesh);
  r.problem_fingerprint = problem_fingerprint(problem);
  r.timings = {t.total_seconds, t.assembly_seconds, t.solver_seconds};
}
} // namespace evolution_detail

inline Evolution evolve(const discretization::TriangleMesh &mesh,
                        const Problem &problem,
                        const InitialConditions &initial,
                        const TimeOptions &options = {}) {
  const auto started = timing::Clock::now();
  TimeGrid grid(options.start, options.end, options.step);
  const std::unique_ptr<TimeIntegrator> integrator =
      options.integrator ? options.integrator()
                         : make_time_integrator(options.method);
  if (!integrator)
    throw std::invalid_argument(
        "Time integrator factory returned no integrator");
  Evolution result;
  if (options.frame_stride == 0)
    throw std::invalid_argument("Frame stride must be positive");
  solver_detail::validate(options.solver);
  if (integrator->order() == 2 && !initial.velocity)
    throw std::invalid_argument(
        "Second-order evolution requires an initial velocity");
  validate_field(mesh, initial.value, "Initial value");
  if (initial.velocity)
    validate_field(mesh, *initial.velocity, "Initial velocity");

  std::unique_ptr<PreparedAssembly> assembly;
  std::unique_ptr<PreparedLinearSolver> prepared;
  auto timed_assembly = [&](double stamp, DirichletDerivative derivative =
                                              DirichletDerivative::Value) {
    timing::Accumulate _(result.timings.assembly_seconds);
    if (!assembly)
      assembly =
          std::make_unique<PreparedAssembly>(mesh, problem, options.assembly);
    auto system = assembly->assemble(stamp, derivative);
    ++result.timings.assembly_calls;
    return system;
  };
  IntegratorServices services;
  services.boundary_tolerance = options.assembly.boundary_tolerance;
  services.solve = [&](const SparseMatrix &a, const Vector &b,
                       const std::vector<DirichletValue> &data,
                       bool time_step) {
    return evolution_detail::solve_step(a, b, data, options.solver, result,
                                        prepared,
                                        time_step && options.store_reports);
  };
  services.start_dirichlet = [&](DirichletDerivative derivative) {
    return timed_assembly(options.start, derivative).dirichlet;
  };
  services.assemble = [&](double stamp) { return timed_assembly(stamp); };

  auto old = timed_assembly(options.start);
  integrator->check(old);
  const auto &execution = options.assembly.execution;
  IntegratorState state{
      nodal_values(mesh, initial.value, options.start, execution),
      initial.velocity
          ? nodal_values(mesh, *initial.velocity, options.start, execution)
          : interpolate(mesh, Expr(0), options.start, execution)};
  impose_compatible(state.u, old.dirichlet, options.assembly.boundary_tolerance,
                    "displacement");
  integrator->start(old, state, services);
  const auto nodes = static_cast<Eigen::Index>(mesh.nodes.size());
  evolution_detail::check_state(state, nodes, "start()");

  auto observe_and_store = [&](double stamp, std::size_t index) {
    const bool retain =
        index == 0 || index % options.frame_stride == 0 || stamp == options.end;
    if (!retain && !options.observer)
      return;
    Frame frame{stamp, state.u, state.v};
    if (options.observer) {
      timing::Accumulate _(result.timings.observer_seconds);
      options.observer(frame);
    }
    if (retain)
      result.frames.push_back(std::move(frame));
  };
  observe_and_store(options.start, 0);
  while (const auto step = grid.next()) {
    auto current = timed_assembly(step->time);
    integrator->check(current);
    auto next = integrator->advance(old, current, step->h, state, services);
    evolution_detail::check_state(next, nodes, "advance()");
    // Avoid roundoff in the public boundary displacement.
    for (const auto &d : current.dirichlet)
      next.u[d.node] = d.value;
    if (!next.u.allFinite() || !next.v.allFinite())
      throw std::runtime_error("Nonfinite evolution output");
    state = std::move(next);
    old = std::move(current);
    ++result.timings.time_steps;
    observe_and_store(step->time, step->index);
  }
  result.timings.total_seconds = timing::seconds_since(started);
  result.timings.other_seconds = std::max(
      0., result.timings.total_seconds - result.timings.assembly_seconds -
              result.timings.constraints_seconds -
              result.timings.solver_seconds - result.timings.observer_seconds);
  evolution_detail::record(result, mesh, problem, initial, options,
                           *integrator);
  return result;
}
} // namespace pde2d::fem
