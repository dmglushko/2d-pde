#pragma once
// Time evolution driver.
//
//   Problem          spatial equation + boundary conditions (problem.hpp)
//   InitialConditions u (and u_t for second-order problems) at the start,
//                     each an Expr or one value per mesh node (Field)
//   TimeOptions       scheme (built-in TimeMethod or a user TimeIntegrator),
//                     interval/step, assembly/solver options, output
//
//   Evolution e = evolve(mesh, problem, {u0, v0}, options);
//   Evolution e = evolve(mesh, problem, {u0, v0}, plan, output);  // plan.hpp
//
// evolve() owns the caching (one PreparedAssembly, reuse of a factorization
// while the constrained matrix is unchanged), output frames, the optional
// observer, work/timing accounting and the checks of what integrators and
// step controllers return. The numerical recurrence of each scheme lives in
// time_integration.hpp, the step sequence in step_control.hpp (default:
// time_grid.hpp). Every step is a trial that the step controller accepts or
// rejects (step_control.hpp); only accepted steps produce frames.
// Evolution::record holds what is needed to reproduce the run (record.hpp):
// the plan with its decisions, the input fingerprints and the strategies
// that ran. EvolutionMetadata is a compatibility view derived from it.
//
// Settings have one source of truth each: TimePlan (plan.hpp) for the time
// discretization, EvolutionOutput for what is kept, AssemblyOptions and
// SolverOptions for the rest. TimeOptions is all four together.
#include "discretization/mesh_fingerprint.hpp"
#include "fem/assembly.hpp"
#include "fem/decision.hpp"
#include "fem/fingerprint.hpp"
#include "fem/interpolation.hpp"
#include "fem/linear_solver.hpp"
#include "fem/plan.hpp"
#include "fem/record.hpp"
#include "fem/solver.hpp"
#include "fem/step_control.hpp"
#include "fem/time_grid.hpp"
#include "fem/time_integration.hpp"
#include "fem/timing.hpp"
#include <functional>
#include <memory>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
struct InitialConditions {
  // u(x, y, start): an expression, or one value per mesh node (for example
  // the last frame of an earlier evolution moved by transfer_field).
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
// What evolve() keeps and reports, separate from the plan of the solve.
struct EvolutionOutput {
  // Retain frames 0, stride, 2*stride, ... and the final frame.
  std::size_t frame_stride = 1;
  bool store_reports = true;
  // Called for every accepted time stamp, retained or not.
  std::function<void(const Frame &)> observer{};
};
// The option-based form of a plan: the time settings (TimePlan: method,
// interval, step, integrator, step_controller, max_consecutive_rejections),
// the output settings (EvolutionOutput: frame_stride, store_reports,
// observer) and the assembly and solver options. Fields are reached as
// before (options.step, options.frame_stride); a new time setting belongs
// in TimePlan and appears here automatically.
struct TimeOptions : TimePlan, EvolutionOutput {
  AssemblyOptions assembly;
  SolverOptions solver;
};
// Deprecated compatibility view of Evolution::record (use the record): the
// same values under their old names, derived by evolution_metadata(), so it
// needs no change when a setting is added.
struct EvolutionMetadata {
  // Selected policy, not an observation of actual OpenMP team sizes. Zero
  // threads delegates the choice to the runtime; small loops may stay serial.
  pde2d::ExecutionOptions execution;
  bool openmp_available = false;
  std::size_t nodes = 0, elements = 0, boundary_edges = 0;
  int quadrature_degree = 2;
  // TimeIntegrator::method() of the integrator that ran.
  TimeMethod method = TimeMethod::BackwardEuler;
  SolverOptions solver;
  double start = 0, end = 0, step = 0;
  // TimeIntegrator::name() of the integrator that ran.
  std::string method_name{};
  // Time integrator, step sequence and linear solver choices (decision.hpp).
  std::vector<Decision> decisions{};
};
// The view of a record. `method` is TimeIntegrator::method() of what ran
// (TimeMethod::Custom for most user schemes), which the record names only.
inline EvolutionMetadata evolution_metadata(const RunRecord &record,
                                            TimeMethod method) {
  EvolutionMetadata m;
  m.execution = record.execution.selected;
  m.openmp_available = record.execution.openmp_available;
  m.nodes = record.workload.nodes;
  m.elements = record.workload.elements;
  m.boundary_edges = record.workload.boundary_edges;
  m.quadrature_degree = record.plan.assembly.quadrature_degree;
  m.method = method;
  m.solver = record.plan.solver;
  if (const auto &t = record.plan.time) {
    m.start = t->start;
    m.end = t->end;
    m.step = t->step;
  }
  for (const auto &strategy : record.strategies)
    if (strategy.subject == DecisionSubject::TimeIntegrator)
      m.method_name = strategy.name;
  m.decisions = record.plan.decisions;
  return m;
}
struct Evolution {
  std::vector<Frame> frames;
  std::vector<SolverReport> reports;
  EvolutionTimings timings;
  // Deprecated: evolution_metadata(record, ...); kept for field access.
  EvolutionMetadata metadata;
  // Plan, inputs, strategies, execution, workload, work and evidence of the
  // run (record.hpp).
  RunRecord record{};
};

// The plan and the output part of TimeOptions.
inline SolvePlan plan_from_user(const TimeOptions &options) {
  return plan_from_user(options.assembly, options.solver,
                        static_cast<const TimePlan &>(options));
}
inline EvolutionOutput output_options(const TimeOptions &options) {
  return static_cast<const EvolutionOutput &>(options);
}

namespace evolution_detail {
// Constrain, then solve with the cached factorization when the constrained
// matrix is coefficient-for-coefficient unchanged. Reports of a trial step
// wait in `pending` until the step is accepted.
inline Vector solve_step(const SparseMatrix &a, const Vector &b,
                         const std::vector<DirichletValue> &data,
                         const SolverOptions &options, Evolution &result,
                         std::unique_ptr<PreparedLinearSolver> &prepared,
                         std::vector<SolverReport> *pending) {
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
  if (pending)
    pending->push_back(solution.report);
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

// A step must continue the sequence, stay inside [start, end] and use an h
// that agrees with its stamps (step_size_consistent).
inline void check_step(const TimeStep &step, std::size_t index, double time,
                       double start, double end) {
  if (step.index != index)
    throw std::invalid_argument(
        "Step controller returned a non-consecutive step index");
  if (!std::isfinite(step.time) || !std::isfinite(step.h) || !(step.h > 0) ||
      !(step.time > time) || step.time > end)
    throw std::invalid_argument("Step controller returned an invalid step");
  if (!step_size_consistent(step, time, start, end))
    throw std::invalid_argument("Step controller returned a step size that "
                                "disagrees with its time stamps");
}

// Name the strategy objects that actually run in their decision records.
inline void name_choice(std::vector<Decision> &decisions,
                        DecisionSubject subject, const std::string &name) {
  if (auto *decision = find_decision(decisions, subject))
    decision->choice = name;
}
inline FieldRecord field_record(const Field &field) {
  return {field_kind(field), field_fingerprint(field)};
}

inline Evolution run(const discretization::TriangleMesh &mesh,
                     const Problem &problem, const InitialConditions &initial,
                     const SolvePlan &plan, const EvolutionOutput &output,
                     const discretization::MeshResult *mesh_result) {
  const auto started = timing::Clock::now();
  if (!plan.time)
    throw std::invalid_argument("Evolution plan has no time settings");
  solver_detail::checked_plan(plan, mesh_result);
  const auto &time = *plan.time;
  const auto &assembly_options = plan.assembly;
  const auto &solver_options = plan.solver;
  Evolution result;
  std::unique_ptr<StepController> controller;
  if (time.step_controller) {
    if (!std::isfinite(time.start) || !std::isfinite(time.end) ||
        time.end < time.start || !std::isfinite(time.end - time.start))
      throw std::invalid_argument("Invalid time interval or step");
    controller = time.step_controller();
    if (!controller)
      throw std::invalid_argument(
          "Step controller factory returned no controller");
  } else
    controller = std::make_unique<UniformStepController>(time.start, time.end,
                                                         time.step);
  const std::unique_ptr<TimeIntegrator> integrator =
      time.integrator ? time.integrator() : make_time_integrator(time.method);
  if (!integrator)
    throw std::invalid_argument(
        "Time integrator factory returned no integrator");
  // Factory-made strategies exist now: their decisions must name them.
  validate_decisions(plan, {integrator->name(), controller->name()});
  auto decisions = plan.decisions;
  name_choice(decisions, DecisionSubject::TimeIntegrator, integrator->name());
  name_choice(decisions, DecisionSubject::TimeSteps, controller->name());
  if (output.frame_stride == 0)
    throw std::invalid_argument("Frame stride must be positive");
  solver_detail::validate(solver_options);
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
          std::make_unique<PreparedAssembly>(mesh, problem, assembly_options);
    auto system = assembly->assemble(stamp, derivative);
    ++result.timings.assembly_calls;
    return system;
  };
  // Step reports of the current trial; appended to result.reports on accept.
  std::vector<SolverReport> pending;
  IntegratorServices services;
  services.boundary_tolerance = assembly_options.boundary_tolerance;
  services.solve = [&](const SparseMatrix &a, const Vector &b,
                       const std::vector<DirichletValue> &data,
                       bool time_step) {
    return evolution_detail::solve_step(
        a, b, data, solver_options, result, prepared,
        time_step && output.store_reports ? &pending : nullptr);
  };
  services.start_dirichlet = [&](DirichletDerivative derivative) {
    return timed_assembly(time.start, derivative).dirichlet;
  };
  services.assemble = [&](double stamp) { return timed_assembly(stamp); };

  auto old = timed_assembly(time.start);
  integrator->check(old);
  const auto &execution = assembly_options.execution;
  IntegratorState state{
      nodal_values(mesh, initial.value, time.start, execution),
      initial.velocity
          ? nodal_values(mesh, *initial.velocity, time.start, execution)
          : interpolate(mesh, Expr(0), time.start, execution)};
  impose_compatible(state.u, old.dirichlet, assembly_options.boundary_tolerance,
                    "displacement");
  integrator->start(old, state, services);
  const auto nodes = static_cast<Eigen::Index>(mesh.nodes.size());
  evolution_detail::check_state(state, nodes, "start()");
  // A user scheme may report start-up solves as time steps.
  for (auto &report : pending)
    result.reports.push_back(std::move(report));
  pending.clear();

  auto observe_and_store = [&](double stamp, std::size_t index) {
    const bool retain =
        index == 0 || index % output.frame_stride == 0 || stamp == time.end;
    if (!retain && !output.observer)
      return;
    Frame frame{stamp, state.u, state.v};
    if (output.observer) {
      timing::Accumulate _(result.timings.observer_seconds);
      output.observer(frame);
    }
    if (retain)
      result.frames.push_back(std::move(frame));
  };
  observe_and_store(time.start, 0);
  double reached = time.start;
  std::size_t consecutive_rejections = 0;
  while (const auto step = controller->next()) {
    evolution_detail::check_step(*step, result.timings.time_steps + 1, reached,
                                 time.start, time.end);
    auto current = timed_assembly(step->time);
    integrator->check(current);
    auto next = integrator->trial(old, current, step->h, state, services);
    evolution_detail::check_state(next, nodes, "advance()");
    // Avoid roundoff in the public boundary displacement.
    for (const auto &d : current.dirichlet)
      next.u[d.node] = d.value;
    if (!next.u.allFinite() || !next.v.allFinite())
      throw std::runtime_error("Nonfinite evolution output");
    const StepOutcome outcome{*step, state, next};
    if (controller->judge(outcome) == StepVerdict::Reject) {
      if (!integrator->transactional())
        throw std::invalid_argument(
            "Step controller rejected a trial step, but time integrator " +
            integrator->name() + " cannot discard one (transactional())");
      integrator->reject();
      pending.clear();
      ++result.timings.rejected_steps;
      if (++consecutive_rejections > time.max_consecutive_rejections)
        throw std::runtime_error(
            "Step controller " + controller->name() + " rejected " +
            std::to_string(consecutive_rejections) +
            " consecutive trial steps at step " + std::to_string(step->index) +
            " (more than TimeOptions::max_consecutive_rejections = " +
            std::to_string(time.max_consecutive_rejections) + ")");
      continue;
    }
    consecutive_rejections = 0;
    integrator->accept();
    controller->accepted(outcome);
    for (auto &report : pending)
      result.reports.push_back(std::move(report));
    pending.clear();
    state = std::move(next);
    old = std::move(current);
    reached = step->time;
    ++result.timings.time_steps;
    observe_and_store(step->time, step->index);
  }
  if (reached != time.end)
    throw std::invalid_argument("Step controller stopped before the end time");
  auto record_plan = plan;
  record_plan.decisions = decisions;
  const auto &t = result.timings;
  auto &record = result.record;
  record = record_detail::make(
      std::move(record_plan), record_detail::workload(mesh),
      discretization::mesh_fingerprint(mesh),
      {t.assembly_calls, t.linear_solves, t.solver.preparations,
       t.solver_reuses, t.time_steps, t.rejected_steps});
  if (mesh_result)
    set_mesh_provenance(record, *mesh_result);
  record.problem_fingerprint = problem_fingerprint(problem);
  record.initial = InitialRecord{field_record(initial.value), std::nullopt};
  if (initial.velocity)
    record.initial->velocity = field_record(*initial.velocity);
  set_strategy(record, strategy_record(DecisionSubject::TimeIntegrator,
                                       integrator->name(), !time.integrator,
                                       integrator->descriptor()));
  set_strategy(
      record, strategy_record(DecisionSubject::TimeSteps, controller->name(),
                              !time.step_controller, controller->descriptor()));
  result.metadata = evolution_metadata(record, integrator->method());
  result.timings.total_seconds = timing::seconds_since(started);
  result.timings.other_seconds = std::max(
      0., result.timings.total_seconds - result.timings.assembly_seconds -
              result.timings.constraints_seconds -
              result.timings.solver_seconds - result.timings.observer_seconds);
  record.timings.total_seconds = result.timings.total_seconds;
  return result;
}
} // namespace evolution_detail

// Evolve with the settings of a plan (plan.hpp); `output` selects what is
// kept. evolve(mesh, problem, initial, TimeOptions) is this function with
// plan_from_user(options) and output_options(options). Decisions are
// validated at entry (validate_decisions): a stale one throws.
inline Evolution evolve(const discretization::TriangleMesh &mesh,
                        const Problem &problem,
                        const InitialConditions &initial, const SolvePlan &plan,
                        const EvolutionOutput &output = {}) {
  return evolution_detail::run(mesh, problem, initial, plan, output, nullptr);
}
inline Evolution evolve(const discretization::TriangleMesh &mesh,
                        const Problem &problem,
                        const InitialConditions &initial,
                        const TimeOptions &options = {}) {
  return evolve(mesh, problem, initial, plan_from_user(options),
                output_options(options));
}
// From a MeshResult: the record also carries its options and report, and
// plan.mesh (if set) must match result.options.
inline Evolution evolve(const discretization::MeshResult &result,
                        const Problem &problem,
                        const InitialConditions &initial, const SolvePlan &plan,
                        const EvolutionOutput &output = {}) {
  return evolution_detail::run(result.mesh, problem, initial, plan, output,
                               &result);
}
inline Evolution evolve(const discretization::MeshResult &result,
                        const Problem &problem,
                        const InitialConditions &initial,
                        const TimeOptions &options = {}) {
  return evolve(result, problem, initial, plan_from_user(options),
                output_options(options));
}
} // namespace pde2d::fem
