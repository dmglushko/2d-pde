#pragma once
// Time evolution driver.
//
//   Problem          spatial equation + boundary conditions (problem.hpp)
//   InitialConditions u (and u_t for second-order problems) at options.start
//   TimeOptions       scheme (built-in TimeMethod or a user TimeIntegrator),
//                     interval/step, assembly/solver options, output
//
//   Evolution e = evolve(mesh, problem, {u0, v0}, options);
//
// evolve() owns the caching (one PreparedAssembly, reuse of a factorization
// while the constrained matrix is unchanged), output frames, the optional
// observer, work/timing accounting and the checks of what integrators and
// step controllers return. The numerical recurrence of each scheme lives in
// time_integration.hpp, the step sequence in step_control.hpp (default:
// time_grid.hpp). EvolutionMetadata::decisions records the choices made.
#include "fem/assembly.hpp"
#include "fem/decision.hpp"
#include "fem/interpolation.hpp"
#include "fem/linear_solver.hpp"
#include "fem/solver.hpp"
#include "fem/step_control.hpp"
#include "fem/time_grid.hpp"
#include "fem/time_integration.hpp"
#include "fem/timing.hpp"
#include <functional>
#include <memory>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::fem {
struct InitialConditions {
  Expr value = 0; // u(x, y, start)
  // u_t(x, y, start). Required by second-order schemes (Newmark). First-order
  // schemes only report it in the initial frame (0 when omitted).
  std::optional<Expr> velocity{};
};
struct Frame {
  double time;
  Vector values;
  Vector velocity;
};
// Creates the integrator of one evolution (integrators may keep state).
using TimeIntegratorFactory = std::function<std::unique_ptr<TimeIntegrator>()>;
struct TimeOptions {
  TimeMethod method = TimeMethod::BackwardEuler;
  double start = 0, end = 1, step = .01;
  AssemblyOptions assembly;
  SolverOptions solver;
  // Retain frames 0, stride, 2*stride, ... and the final frame.
  std::size_t frame_stride = 1;
  bool store_reports = true;
  // Called for every time stamp, retained or not.
  std::function<void(const Frame &)> observer{};
  // A user scheme. When set, evolve() calls it once and ignores `method`.
  TimeIntegratorFactory integrator{};
  // A user step sequence on [start, end]. When set, evolve() calls it once
  // and ignores `step`; otherwise steps are uniform (UniformStepController).
  StepControllerFactory step_controller{};
};
struct EvolutionMetadata {
  // Selected policy, not an observation of actual OpenMP team sizes. Zero
  // threads delegates the choice to the runtime; small loops may stay serial.
  pde2d::ExecutionOptions execution;
  bool openmp_available = false;
  std::size_t nodes = 0, elements = 0, boundary_edges = 0;
  int quadrature_degree = 2;
  TimeMethod method = TimeMethod::BackwardEuler;
  SolverOptions solver;
  double start = 0, end = 0, step = 0;
  // TimeIntegrator::name() of the integrator that ran.
  std::string method_name{};
  // Time integrator, step sequence and linear solver choices (decision.hpp).
  std::vector<Decision> decisions{};
};
struct Evolution {
  std::vector<Frame> frames;
  std::vector<SolverReport> reports;
  EvolutionTimings timings;
  EvolutionMetadata metadata;
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

// A step must continue the sequence and stay inside [start, end].
inline void check_step(const TimeStep &step, std::size_t index, double time,
                       double end) {
  if (step.index != index)
    throw std::invalid_argument(
        "Step controller returned a non-consecutive step index");
  if (!std::isfinite(step.time) || !std::isfinite(step.h) || !(step.h > 0) ||
      !(step.time > time) || step.time > end)
    throw std::invalid_argument("Step controller returned an invalid step");
}
} // namespace evolution_detail

inline Evolution evolve(const discretization::TriangleMesh &mesh,
                        const Problem &problem,
                        const InitialConditions &initial,
                        const TimeOptions &options = {}) {
  const auto started = timing::Clock::now();
  Evolution result;
  result.metadata = {options.assembly.execution,
                     pde2d::openmp_available(),
                     mesh.nodes.size(),
                     mesh.elements.size(),
                     mesh.boundary_edges.size(),
                     options.assembly.quadrature_degree,
                     options.method,
                     options.solver,
                     options.start,
                     options.end,
                     options.step};
  std::unique_ptr<StepController> controller;
  if (options.step_controller) {
    if (!std::isfinite(options.start) || !std::isfinite(options.end) ||
        options.end < options.start ||
        !std::isfinite(options.end - options.start))
      throw std::invalid_argument("Invalid time interval or step");
    controller = options.step_controller();
    if (!controller)
      throw std::invalid_argument(
          "Step controller factory returned no controller");
  } else
    controller = std::make_unique<UniformStepController>(
        options.start, options.end, options.step);
  const std::unique_ptr<TimeIntegrator> integrator =
      options.integrator ? options.integrator()
                         : make_time_integrator(options.method);
  if (!integrator)
    throw std::invalid_argument(
        "Time integrator factory returned no integrator");
  result.metadata.method = integrator->method();
  result.metadata.method_name = integrator->name();
  auto &decisions = result.metadata.decisions;
  decisions.push_back(user_decision(
      "time integrator", integrator->name(),
      options.integrator ? "TimeOptions::integrator" : "TimeOptions::method"));
  decisions.push_back(user_decision("time steps", controller->name(),
                                    options.step_controller
                                        ? "TimeOptions::step_controller"
                                        : "TimeOptions::step"));
  for (auto &decision : solver_decisions(options.solver))
    decisions.push_back(std::move(decision));
  if (options.frame_stride == 0)
    throw std::invalid_argument("Frame stride must be positive");
  solver_detail::validate(options.solver);
  if (integrator->order() == 2 && !initial.velocity)
    throw std::invalid_argument(
        "Second-order evolution requires an initial velocity");
  require_symbols(initial.value, field_symbols, "Initial value");
  if (initial.velocity)
    require_symbols(*initial.velocity, field_symbols, "Initial velocity");

  std::unique_ptr<PreparedAssembly> assembly;
  std::unique_ptr<PreparedLinearSolver> prepared;
  auto timed_assembly = [&](double time, DirichletDerivative derivative =
                                             DirichletDerivative::Value) {
    timing::Accumulate _(result.timings.assembly_seconds);
    if (!assembly)
      assembly =
          std::make_unique<PreparedAssembly>(mesh, problem, options.assembly);
    auto system = assembly->assemble(time, derivative);
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
  services.assemble = [&](double time) { return timed_assembly(time); };

  auto old = timed_assembly(options.start);
  integrator->check(old);
  const auto &execution = options.assembly.execution;
  IntegratorState state{
      interpolate(mesh, initial.value, options.start, execution),
      interpolate(mesh, initial.velocity.value_or(Expr(0)), options.start,
                  execution)};
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
  double reached = options.start;
  while (const auto step = controller->next()) {
    evolution_detail::check_step(*step, result.timings.time_steps + 1, reached,
                                 options.end);
    auto current = timed_assembly(step->time);
    integrator->check(current);
    auto next = integrator->advance(old, current, step->h, state, services);
    evolution_detail::check_state(next, nodes, "advance()");
    // Avoid roundoff in the public boundary displacement.
    for (const auto &d : current.dirichlet)
      next.u[d.node] = d.value;
    if (!next.u.allFinite() || !next.v.allFinite())
      throw std::runtime_error("Nonfinite evolution output");
    controller->accepted({*step, state, next});
    state = std::move(next);
    old = std::move(current);
    reached = step->time;
    ++result.timings.time_steps;
    observe_and_store(step->time, step->index);
  }
  if (reached != options.end)
    throw std::invalid_argument("Step controller stopped before the end time");
  result.timings.total_seconds = timing::seconds_since(started);
  result.timings.other_seconds = std::max(
      0., result.timings.total_seconds - result.timings.assembly_seconds -
              result.timings.constraints_seconds -
              result.timings.solver_seconds - result.timings.observer_seconds);
  return result;
}
} // namespace pde2d::fem
