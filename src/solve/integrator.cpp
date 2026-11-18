#include "pde2d/solve/integrator.hpp"

#include "pde2d/fem/assembler.hpp"

#include "requirements.hpp"
#include "vectors.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::solve {

namespace {

using Clock = std::chrono::steady_clock;

// A step that would end within this fraction of dt of a target time (a
// snapshot time or t1) lands on the target instead, so that rounding in
// the times never leaves a tiny last step.
constexpr double landing_tolerance = 1e-9;

void check_output(const IntegrationOutput &output, double t0, double t1) {
  const auto &times = output.snapshot_times;
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || times[i] < t0 || times[i] > t1)
      throw std::invalid_argument("integrator: snapshot time " +
                                  std::to_string(times[i]) +
                                  " is not within [t0, t1]");
    if (i > 0 && !(times[i] > times[i - 1]))
      throw std::invalid_argument(
          "integrator: snapshot times must be strictly increasing");
  }
}

// The entries of `all` (one per mesh node) at the free nodes.
std::vector<double> free_part(const fem::MeshedProblem &meshed,
                              const std::vector<double> &all) {
  std::vector<double> part;
  part.reserve(meshed.free_nodes().size());
  for (const std::size_t node : meshed.free_nodes())
    part.push_back(all[node]);
  return part;
}

TimeState starting_state(const fem::MeshedProblem &meshed, double t0,
                         int time_order, const Solution *initial) {
  TimeState state;
  if (initial) {
    const std::size_t nodes = meshed.mesh()->nodes.size();
    if (initial->mesh != meshed.mesh())
      throw std::invalid_argument(
          "integrator: the initial solution is on another mesh object");
    if (initial->values.size() != nodes)
      throw std::invalid_argument("integrator: the initial solution has " +
                                  std::to_string(initial->values.size()) +
                                  " values for " + std::to_string(nodes) +
                                  " nodes");
    if (!detail::all_finite(initial->values))
      throw std::invalid_argument(
          "integrator: the initial solution has a value that is not finite");
    if (time_order == 0)
      return state; // checked only: the state at t0 is solved for
    state.values = free_part(meshed, initial->values);
    if (time_order == 2) {
      if (initial->velocities.size() != nodes)
        throw std::invalid_argument(
            "integrator: time order 2 needs an initial solution with "
            "velocities, one per node");
      state.velocities = free_part(meshed, initial->velocities);
    }
  } else if (time_order >= 1) {
    // The problem's own time order is at least the assembled one, so the
    // values (and for time order 2 the velocities) exist.
    // Only the free nodes: the boundary data at t0 (and its time
    // derivative) is never read here, so a method whose start needs
    // nothing at t0 never evaluates it.
    auto initial_state = *meshed.initial_free_state(t0);
    state.values = std::move(initial_state.values);
    if (time_order == 2)
      state.velocities = std::move(initial_state.velocities);
  }
  if (!detail::all_finite(state.values) ||
      !detail::all_finite(state.velocities))
    throw std::invalid_argument(
        "integrator: the initial state is not finite at a free node");
  return state;
}

// weights.mass M2 + weights.damping B + weights.stiffness A on the free
// pattern, which the three operators share.
std::shared_ptr<const fem::CsrMatrix>
step_matrix(const fem::AssembledProblem &problem,
            const StepMatrixWeights &weights) {
  fem::CsrMatrix matrix;
  matrix.pattern = problem.stiffness().free.pattern;
  matrix.values.assign(matrix.pattern->nonzeros(), 0.0);
  const auto add = [&](const fem::Operator &op, double weight) {
    if (op.always_zero || weight == 0.0)
      return;
    for (std::size_t s = 0; s < matrix.values.size(); ++s)
      matrix.values[s] += weight * op.free.values[s];
  };
  add(problem.mass(), weights.mass);
  add(problem.damping(), weights.damping);
  add(problem.stiffness(), weights.stiffness);
  return std::make_shared<const fem::CsrMatrix>(std::move(matrix));
}

// Whether the step matrix built from `problem` differs from one built from
// an earlier assembly with the same weights: an operator in it (a non-zero
// weight, not always zero) depends on t. Formula facts only: an operator
// that is zero now may not be zero at the next step.
bool step_matrix_depends_on_t(const fem::AssembledProblem &problem,
                              const StepMatrixWeights &weights) {
  const auto changes = [](const fem::Operator &op, double weight) {
    return weight != 0.0 && !op.always_zero && op.depends_on_t;
  };
  return changes(problem.mass(), weights.mass) ||
         changes(problem.damping(), weights.damping) ||
         changes(problem.stiffness(), weights.stiffness);
}

Solution make_solution(const fem::MeshedProblem &meshed, const TimeState &state,
                       int time_order, double t) {
  Solution solution = build_solution(meshed, state.values, t);
  if (time_order == 2) {
    solution.velocities.assign(solution.values.size(), 0.0);
    const auto &free_nodes = meshed.free_nodes();
    for (std::size_t i = 0; i < free_nodes.size(); ++i)
      solution.velocities[free_nodes[i]] = state.velocities[i];
    const auto &dirichlet_nodes = meshed.dirichlet_nodes();
    for (std::size_t k = 0; k < dirichlet_nodes.size(); ++k)
      solution.velocities[dirichlet_nodes[k].node] =
          meshed.dirichlet_rate(k, t);
  }
  return solution;
}

// What the integrator keeps of every linear solve of one integrate()
// call, also of the solves a method's start() does: the warnings
// (LinearSolveResult::messages), each distinct one once (a solver
// prepared for a negative definite step matrix warns at every step), in
// the order they were added; the largest backward error; and the
// smallest measured pivot ratio (one above 0; 0 is "not measured"). A
// time method may solve from several threads at once (a prepared solver
// is const and may be used concurrently), so the log is guarded by a
// mutex; it is held only to add or copy, never during a solve. Solves
// that overlap add their messages in the order they take the lock; a
// warning's text says which solver and matrix it is about, so that order
// carries no information the report needs (the largest and smallest
// values do not depend on it).
class SolveLog {
public:
  void add(const LinearSolveResult &result) {
    const std::scoped_lock lock(mutex_);
    for (const std::string &message : result.messages)
      if (std::ranges::find(messages_, message) == messages_.end())
        messages_.push_back(message);
    // A NaN backward error stays (solvers that check it throw first).
    if (!std::isnan(max_backward_error_) &&
        !(result.backward_error <= max_backward_error_))
      max_backward_error_ = result.backward_error;
    if (result.pivot_ratio > 0.0)
      min_pivot_ratio_ = std::min(min_pivot_ratio_, result.pivot_ratio);
  }
  // What the report shows, read under one lock so that its three parts
  // agree (a prepared solver kept beyond integrate() may finish a solve
  // at any time).
  struct Snapshot {
    std::vector<std::string> messages;
    // 0 before any solve.
    double max_backward_error = 0.0;
    // 0 when no solve measured its pivot ratio (an iterative solver,
    // SparseLu, an empty system, no solve).
    double min_pivot_ratio = 0.0;
  };
  Snapshot snapshot() const {
    const std::scoped_lock lock(mutex_);
    return {.messages = messages_,
            .max_backward_error = max_backward_error_,
            .min_pivot_ratio =
                std::isinf(min_pivot_ratio_) ? 0.0 : min_pivot_ratio_};
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::string> messages_;
  double max_backward_error_ = 0.0;
  double min_pivot_ratio_ = std::numeric_limits<double>::infinity();
};

// A prepared solver that passes every solve on and logs its result. It
// shares the log of its integrate() call: a method may keep a prepared
// solver beyond that call (from its start() or a step) and solve with it
// later, which still works and adds to a log that no report reads any
// more.
class LoggingPreparedSolver final : public PreparedLinearSolver {
public:
  LoggingPreparedSolver(std::unique_ptr<const PreparedLinearSolver> solver,
                        std::shared_ptr<SolveLog> log)
      : solver_(std::move(solver)), log_(std::move(log)) {}
  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    LinearSolveResult result = solver_->solve(right_hand_side);
    log_->add(result);
    return result;
  }

private:
  std::unique_ptr<const PreparedLinearSolver> solver_;
  std::shared_ptr<SolveLog> log_;
};

// The integrator's linear solver as the time method sees it: the same
// solver, whose prepared solvers log every solve, also the solves a
// method's start() does (Newmark's acceleration, the state at t0 for time
// order 0). So the messages, the largest backward error and the smallest
// pivot ratio reach the report whatever a method does with a
// LinearSolveResult.
class LoggingSolver final : public LinearSolver {
public:
  LoggingSolver(std::shared_ptr<const LinearSolver> solver,
                std::shared_ptr<SolveLog> log)
      : solver_(std::move(solver)), log_(std::move(log)) {}
  std::string name() const override { return solver_->name(); }
  LinearSolverRequirements requirements() const override {
    return solver_->requirements();
  }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override {
    return std::make_unique<const LoggingPreparedSolver>(
        solver_->prepare(std::move(matrix)), log_);
  }

private:
  std::shared_ptr<const LinearSolver> solver_;
  std::shared_ptr<SolveLog> log_;
};

// The running summary of the step records.
struct Summary {
  std::size_t steps = 0;
  std::size_t rejected = 0;
  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = 0.0;
  std::size_t linear_iterations = 0;
  std::size_t max_linear_iterations = 0;
  double max_residual = 0.0;
  std::size_t linear_threads = 0;

  void add(const StepRecord &record, std::size_t threads) {
    linear_threads = std::max(linear_threads, threads);
    ++steps;
    if (!record.accepted)
      ++rejected;
    min_dt = std::min(min_dt, record.dt);
    max_dt = std::max(max_dt, record.dt);
    linear_iterations += record.linear_iterations;
    max_linear_iterations =
        std::max(max_linear_iterations, record.linear_iterations);
    max_residual = std::max(max_residual, record.residual);
  }
};

} // namespace

Integrator::Integrator(std::shared_ptr<const TimeMethod> method,
                       std::shared_ptr<const StepController> step_controller,
                       std::shared_ptr<const LinearSolver> linear_solver,
                       fem::AssemblerOptions assembler_options)
    : method_(std::move(method)), step_controller_(std::move(step_controller)),
      linear_solver_(std::move(linear_solver)), assembler_(assembler_options) {
  if (!method_)
    throw std::invalid_argument("integrator: the time method is null");
  if (!step_controller_)
    throw std::invalid_argument("integrator: the step controller is null");
  if (!linear_solver_)
    throw std::invalid_argument("integrator: the linear solver is null");
}

IntegrationResult
Integrator::integrate(std::shared_ptr<const fem::MeshedProblem> meshed_problem,
                      double t0, double t1, const IntegrationOutput &output,
                      const Solution *initial,
                      std::optional<std::size_t> max_steps) const {
  if (!meshed_problem)
    throw std::invalid_argument("integrator: the problem is null");
  if (!std::isfinite(t0) || !std::isfinite(t1))
    throw std::invalid_argument("integrator: t0 and t1 must be finite");
  if (!(t1 > t0))
    throw std::invalid_argument("integrator: t1 must be greater than t0");
  check_output(output, t0, t1);
  if (max_steps && *max_steps == 0)
    throw std::invalid_argument("integrator: max_steps must be at least 1");
  const fem::MeshedProblem &meshed = *meshed_problem;
  const auto start = Clock::now();
  const auto log = std::make_shared<SolveLog>();
  const LoggingSolver linear_solver(linear_solver_, log);

  // 1. The time order, from assembly's formula facts without evaluating
  // anything (d/dt sqrt(t) may be infinite at t0), against the method's
  // declaration.
  const fem::Assembler &assembler = assembler_;
  const int time_order = assembler.time_order(meshed);
  if (!method_->capabilities().supports(time_order)) {
    std::string message = method_->name() + " does not support time order " +
                          std::to_string(time_order);
    const int written = fem::time_order(meshed);
    if (written > time_order)
      message += std::format(
          "; the problem is written with time order {}, but its time terms "
          "vanished in assembly (for example a boundary u_t term on a "
          "problem with A = 0, whose boundary terms are all zero)",
          written);
    throw std::invalid_argument("integrator: " + message);
  }

  // 2. The starting state. Time order 0 has no initial conditions: the
  // method's start computes the state at t0. The problem is assembled at
  // t0 only as far as the start reads it (docs/decisions.md, "A time
  // method declares what its start reads at t0"); the steps assemble at
  // their own times, so U_D' and U_D'' are never evaluated at t0 for a
  // method whose start reads nothing.
  TimeState state = starting_state(meshed, t0, time_order, initial);
  std::optional<fem::AssembledProblem> current;
  std::size_t assemblies = 0;
  const StartNeeds needs = method_->start_needs(time_order);
  if (needs != StartNeeds::nothing) {
    const int max_time_order = needs == StartNeeds::static_part ? 0 : 2;
    current = assembler.assemble(meshed_problem, t0, nullptr, max_time_order);
    ++assemblies;
    method_->start(*current, linear_solver, state);
  }
  // A method of time order 0 must compute the state at t0 in its start (it
  // has no initial conditions); one that does not is a mistake in that
  // method, found here rather than as a wrong-sized snapshot.
  if (state.values.size() != meshed.free_nodes().size())
    throw std::runtime_error(std::format(
        "integrator: {} gave no state at t0 = {} for time order {} (its "
        "start must compute it)",
        method_->name(), t0, time_order));

  IntegrationResult result;
  std::size_t next_snapshot = 0; // into output.snapshot_times
  if (!output.snapshot_times.empty() && output.snapshot_times[0] == t0) {
    result.snapshots.push_back(make_solution(meshed, state, time_order, t0));
    ++next_snapshot;
  }

  // 3. The steps. On a stretch of equal steps from `anchor`, step k ends at
  // anchor + k dt; the stretch restarts after landing on a target or when
  // the controller changes dt.
  const double fraction = method_->evaluation_fraction();
  double t = t0;
  double anchor = t0;
  std::size_t anchor_steps = 0;
  double anchor_dt = 0.0;
  std::unique_ptr<const PreparedLinearSolver> prepared;
  double prepared_dt = 0.0;
  std::size_t prepares = 0;
  Summary summary;

  while (t < t1 && !(max_steps && summary.steps >= *max_steps)) {
    const auto step_start = Clock::now();
    const double dt = step_controller_->next_step(t);
    if (!std::isfinite(dt) || !(dt > 0.0))
      throw std::runtime_error("integrator: " + step_controller_->name() +
                               " gave a step that is not finite and positive");
    if (dt != anchor_dt) {
      anchor = t;
      anchor_steps = 0;
      anchor_dt = dt;
    }
    const double target = next_snapshot < output.snapshot_times.size()
                              ? output.snapshot_times[next_snapshot]
                              : t1;
    double t_new = anchor + static_cast<double>(anchor_steps + 1) * dt;
    double h = dt;
    bool landed = false;
    if (t_new >= target - landing_tolerance * dt) {
      // Land on the target. The step keeps the nominal dt when it differs
      // only by rounding, so the step matrix does not change for nothing.
      landed = true;
      t_new = target;
      if (std::abs((target - t) - dt) > landing_tolerance * dt)
        h = target - t;
      anchor = target;
      anchor_steps = 0;
    } else {
      ++anchor_steps;
    }
    // A dt below the spacing of doubles near t gives t_new == t: the state
    // would change while the time stands still.
    if (!(t_new > t))
      throw std::runtime_error(std::format(
          "integrator: the step {} at t = {} is too small to change t in "
          "double precision",
          dt, t));

    // The problem at the method's evaluation point (for fraction 1 exactly
    // t_new), reusing the previous assembly.
    const double t_eval = (1.0 - fraction) * t + fraction * t_new;
    // (Reuse gives the same values as a fresh assembly, so a first step
    // without an assembly at t0 gets the same result.)
    current = assembler.assemble(meshed_problem, t_eval,
                                 current ? &*current : nullptr);
    ++assemblies;

    const StepMatrixWeights weights = method_->step_matrix_weights(h);
    if (!prepared || h != prepared_dt ||
        step_matrix_depends_on_t(*current, weights)) {
      auto matrix = step_matrix(*current, weights);
      detail::check_step_matrix_requirements(
          linear_solver, *current, weights, *matrix, "integrator",
          "the step matrix of " + method_->name());
      prepared = linear_solver.prepare(std::move(matrix));
      prepared_dt = h;
      ++prepares;
    }
    // The last step of the run: it lands on t1, or it is the max_steps-th.
    const bool last =
        t_new == t1 || (max_steps && summary.steps + 1 == *max_steps);
    if (last && output.keep_before_last_step)
      result.before_last_step = make_solution(meshed, state, time_order, t);
    const StepSolveInfo solved =
        method_->advance(*current, h, *prepared, state);
    t = t_new;

    const StepRecord record{.t = t,
                            .dt = h,
                            .accepted = true,
                            .linear_iterations = solved.linear_iterations,
                            .residual = solved.residual,
                            .time = Clock::now() - step_start};
    summary.add(record, solved.linear_threads);
    if (output.keep_step_history)
      result.history.push_back(record);

    bool take_snapshot = false;
    if (landed && next_snapshot < output.snapshot_times.size() &&
        t == output.snapshot_times[next_snapshot]) {
      take_snapshot = true;
      ++next_snapshot;
    }
    if (output.snapshot_every > 0 && summary.steps % output.snapshot_every == 0)
      take_snapshot = true;
    if (take_snapshot)
      result.snapshots.push_back(make_solution(meshed, state, time_order, t));
  }

  // t is t1 unless max_steps ended the run first (the last step landed on
  // t1 exactly otherwise).
  result.final = make_solution(meshed, state, time_order, t);

  auto &report = result.report;
  report.stage = "integrator";
  auto &m = report.metrics;
  m["t0"] = t0;
  m["t1"] = t1;
  m["t_end"] = t;
  m["time_order"] = time_order;
  m["steps"] = static_cast<double>(summary.steps);
  m["rejected_steps"] = static_cast<double>(summary.rejected);
  m["min_dt"] = summary.min_dt;
  m["max_dt"] = summary.max_dt;
  m["linear_iterations"] = static_cast<double>(summary.linear_iterations);
  m["max_linear_iterations"] =
      static_cast<double>(summary.max_linear_iterations);
  m["max_residual"] = summary.max_residual;
  m["assemblies"] = static_cast<double>(assemblies);
  m["prepares"] = static_cast<double>(prepares);
  m["snapshots"] = static_cast<double>(result.snapshots.size());
  // Every run assembles at least once (t1 > t0 gives a step); the count is
  // read from the assembly's own report.
  m["assembly_threads"] = current->report().metrics.at("threads");
  m["linear_threads"] = static_cast<double>(summary.linear_threads);
  // Over every solve of the run, start() included (the log); the pivot
  // ratio over the solves that measured it, 0 when none did.
  SolveLog::Snapshot solves = log->snapshot();
  m["max_backward_error"] = solves.max_backward_error;
  m["min_pivot_ratio"] = solves.min_pivot_ratio;
  report.messages = std::move(solves.messages);
  report.time = Clock::now() - start;
  return result;
}

} // namespace pde2d::solve
