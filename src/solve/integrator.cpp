#include "pde2d/solve/integrator.hpp"

#include "pde2d/fem/assembler.hpp"

#include "vectors.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

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
    state.values = free_part(meshed, initial->values);
    if (time_order == 2) {
      if (initial->velocities.size() != nodes)
        throw std::invalid_argument(
            "integrator: time order 2 needs an initial solution with "
            "velocities, one per node");
      state.velocities = free_part(meshed, initial->velocities);
    }
  } else {
    // The problem's own time order is at least the assembled one, so the
    // values (and for time order 2 the velocities) exist.
    state.values = free_part(meshed, *meshed.initial_values(t0));
    if (time_order == 2)
      state.velocities = free_part(meshed, *meshed.initial_velocities(t0));
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
    if (op.is_zero || weight == 0.0)
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
// weight, not a zero operator) depends on t.
bool step_matrix_depends_on_t(const fem::AssembledProblem &problem,
                              const StepMatrixWeights &weights) {
  const auto changes = [](const fem::Operator &op, double weight) {
    return weight != 0.0 && !op.is_zero && op.depends_on_t;
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

// The running summary of the step records.
struct Summary {
  std::size_t steps = 0;
  std::size_t rejected = 0;
  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = 0.0;
  std::size_t linear_iterations = 0;
  std::size_t max_linear_iterations = 0;
  double max_residual = 0.0;

  void add(const StepRecord &record) {
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
                       std::shared_ptr<const LinearSolver> linear_solver)
    : method_(std::move(method)), step_controller_(std::move(step_controller)),
      linear_solver_(std::move(linear_solver)) {
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
                      const Solution *initial) const {
  if (!meshed_problem)
    throw std::invalid_argument("integrator: the problem is null");
  if (!std::isfinite(t0) || !std::isfinite(t1))
    throw std::invalid_argument("integrator: t0 and t1 must be finite");
  if (!(t1 > t0))
    throw std::invalid_argument("integrator: t1 must be greater than t0");
  check_output(output, t0, t1);
  const fem::MeshedProblem &meshed = *meshed_problem;
  const auto start = Clock::now();

  // 1. The time order, from assembly's own facts, against the method's
  // declaration.
  const fem::Assembler assembler;
  fem::AssembledProblem current = assembler.assemble(meshed_problem, t0);
  std::size_t assemblies = 1;
  const int time_order = current.time_order();
  if (time_order == 0)
    throw std::invalid_argument(
        "integrator: the assembled problem has time order 0 (no time "
        "derivative); use StaticSolver");
  const TimeMethodCapabilities capabilities = method_->capabilities();
  if ((time_order == 1 && !capabilities.supports_first_order) ||
      (time_order == 2 && !capabilities.supports_second_order))
    throw std::invalid_argument(method_->name() +
                                " does not support time order " +
                                std::to_string(time_order));

  // 2. The starting state.
  TimeState state = starting_state(meshed, t0, time_order, initial);
  method_->start(current, *linear_solver_, state);

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

  while (t < t1) {
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
    current = assembler.assemble(meshed_problem, t_eval, &current);
    ++assemblies;

    const StepMatrixWeights weights = method_->step_matrix_weights(h);
    if (!prepared || h != prepared_dt ||
        step_matrix_depends_on_t(current, weights)) {
      prepared = linear_solver_->prepare(step_matrix(current, weights));
      prepared_dt = h;
      ++prepares;
    }
    const StepSolveInfo solved = method_->advance(current, h, *prepared, state);
    t = t_new;

    const StepRecord record{.t = t,
                            .dt = h,
                            .accepted = true,
                            .linear_iterations = solved.linear_iterations,
                            .residual = solved.residual,
                            .time = Clock::now() - step_start};
    summary.add(record);
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

  result.final = make_solution(meshed, state, time_order, t1);

  auto &report = result.report;
  report.stage = "integrator";
  auto &m = report.metrics;
  m["t0"] = t0;
  m["t1"] = t1;
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
  report.time = Clock::now() - start;
  return result;
}

} // namespace pde2d::solve
