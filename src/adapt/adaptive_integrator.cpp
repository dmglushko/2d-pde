#include "pde2d/adapt/adaptive_integrator.hpp"

#include "pde2d/adapt/solution_transfer.hpp"
#include "pde2d/fem/assembler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::adapt {

namespace {

constexpr const char *stage = "adaptive_integrator";

// An adaptation time within this fraction of the interval of t1 is t1,
// and one that close to a snapshot time is that time, so that rounding in
// the sums t_k + interval never leaves a tiny piece or step (the
// Integrator's landing rule for its steps).
constexpr double landing_tolerance = 1e-9;

void check_positive(const std::optional<double> &value, const char *name) {
  if (value && !(std::isfinite(*value) && *value > 0.0))
    throw std::invalid_argument(std::format(
        "{}: {} must be finite and positive, got {}", stage, name, *value));
}

void validate(const AdaptiveTimeTarget &target) {
  if (!target.every.interval && !target.every.steps)
    throw std::invalid_argument(std::format(
        "{}: the schedule needs an interval, a number of steps or both",
        stage));
  check_positive(target.every.interval, "every.interval");
  if (target.every.steps && *target.every.steps == 0)
    throw std::invalid_argument(
        std::format("{}: every.steps must be at least 1", stage));
  check_positive(target.tolerance.absolute, "tolerance.absolute");
  check_positive(target.tolerance.relative, "tolerance.relative");
  if (target.max_nodes && *target.max_nodes == 0)
    throw std::invalid_argument(
        std::format("{}: max_nodes must be at least 1", stage));
  if (target.max_refinements && *target.max_refinements == 0)
    throw std::invalid_argument(
        std::format("{}: max_refinements must be at least 1", stage));
}

void validate(const solve::IntegrationOutput &output, double t0, double t1) {
  if (output.snapshot_every != 0)
    throw std::invalid_argument(std::format(
        "{}: snapshot_every is not supported (it counts the steps of one "
        "integrate() call, and the run is integrated in pieces); use "
        "snapshot_times",
        stage));
  const auto &times = output.snapshot_times;
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || times[i] < t0 || times[i] > t1)
      throw std::invalid_argument(std::format(
          "{}: snapshot time {} is not within [t0, t1]", stage, times[i]));
    if (i > 0 && !(times[i] > times[i - 1]))
      throw std::invalid_argument(
          std::format("{}: snapshot times must be strictly increasing", stage));
  }
}

// The smaller of the set values; infinity when neither is set.
double resolve(const ErrorTolerance &tolerance, double first_estimate) {
  double bound = std::numeric_limits<double>::infinity();
  if (tolerance.absolute)
    bound = std::min(bound, *tolerance.absolute);
  if (tolerance.relative)
    bound = std::min(bound, *tolerance.relative * first_estimate);
  return bound;
}

// The time derivatives at the end of a piece, at every node. At the free
// nodes from the last step: u_t = (u(t) - u(t - dt)) / dt, or for time
// order 2 u_t = the velocities and u_tt = their difference quotient. At
// the Dirichlet nodes from the boundary data: for time order 1 (or 0) the
// quotient of its values at t and t - dt; for time order 2 u_t is the
// velocity (its rate at t) and u_tt its acceleration at t, so that no rate
// is evaluated at t - dt (u = sqrt(t) has none at t0 = 0).
struct Derivatives {
  std::vector<double> u_t, u_tt;
};

Derivatives last_step_derivatives(const fem::MeshedProblem &problem,
                                  const solve::Solution &end,
                                  const solve::FreeState &before) {
  const double dt = end.time - before.time;
  const auto &free_nodes = problem.free_nodes();
  const auto &dirichlet = problem.dirichlet_nodes();
  const auto quotient = [&](const std::vector<double> &now,
                            const std::vector<double> &then) {
    std::vector<double> rate(now.size(), 0.0);
    for (std::size_t i = 0; i < free_nodes.size(); ++i)
      rate[free_nodes[i]] = (now[free_nodes[i]] - then[i]) / dt;
    return rate;
  };
  Derivatives d;
  if (!end.velocities.empty()) {
    d.u_t = end.velocities;
    d.u_tt = quotient(end.velocities, before.velocities);
    for (std::size_t k = 0; k < dirichlet.size(); ++k)
      d.u_tt[dirichlet[k].node] = problem.dirichlet_acceleration(k, end.time);
  } else {
    d.u_t = quotient(end.values, before.values);
    for (std::size_t k = 0; k < dirichlet.size(); ++k) {
      const std::size_t node = dirichlet[k].node;
      d.u_t[node] =
          (end.values[node] - problem.dirichlet_value(k, before.time)) / dt;
    }
  }
  return d;
}

} // namespace

std::string_view to_string(AdaptationOutcome outcome) {
  switch (outcome) {
  case AdaptationOutcome::refined:
    return "refined";
  case AdaptationOutcome::tolerance_met:
    return "tolerance_met";
  case AdaptationOutcome::nothing_changed:
    return "nothing_changed";
  case AdaptationOutcome::node_budget:
    return "node_budget";
  }
  return "unknown";
}

AdaptiveIntegrator::AdaptiveIntegrator(
    solve::Integrator integrator,
    std::vector<std::shared_ptr<const Adaptation>> adaptations,
    AdaptiveIntegratorOptions options)
    : integrator_(std::move(integrator)), adaptations_(std::move(adaptations)),
      options_(std::move(options)) {
  if (options_.damping_steps > 0) {
    if (!options_.damping_method)
      throw std::invalid_argument(
          std::format("{}: {} damping steps need a damping method", stage,
                      options_.damping_steps));
    damping_ = integrator_.with_method(options_.damping_method);
  }
  if (adaptations_.empty())
    throw std::invalid_argument(
        std::format("{}: at least one adaptation is needed", stage));
  std::vector<std::string> changing;
  for (std::size_t i = 0; i < adaptations_.size(); ++i) {
    if (!adaptations_[i])
      throw std::invalid_argument(
          std::format("{}: adaptation {} is null", stage, i));
    if (adaptations_[i]->capabilities().changes_mesh)
      changing.push_back(adaptations_[i]->name());
  }
  if (changing.size() > 1)
    throw std::invalid_argument(std::format(
        "{}: at most one adaptation may change the mesh (all act on the "
        "same solved state); {} and {} both do",
        stage, changing[0], changing[1]));
}

AdaptiveIntegrationResult AdaptiveIntegrator::integrate(
    std::shared_ptr<const fem::MeshedProblem> problem, double t0, double t1,
    const AdaptiveTimeTarget &target, const solve::IntegrationOutput &output,
    const solve::Solution *initial) const {
  if (!problem)
    throw std::invalid_argument(std::format("{}: the problem is null", stage));
  if (!std::isfinite(t0) || !std::isfinite(t1))
    throw std::invalid_argument(
        std::format("{}: t0 and t1 must be finite", stage));
  if (!(t1 > t0))
    throw std::invalid_argument(
        std::format("{}: t1 must be greater than t0", stage));
  validate(target);
  validate(output, t0, t1);
  const Adaptation *estimating = nullptr;
  for (const auto &adaptation : adaptations_)
    if (!estimating && adaptation->capabilities().estimates_error)
      estimating = adaptation.get();
  const bool has_tolerance =
      target.tolerance.absolute || target.tolerance.relative;
  if (has_tolerance && !estimating)
    throw std::invalid_argument(std::format(
        "{}: a tolerance needs an adaptation that estimates the error", stage));
  if (damping_) {
    // From the formula facts, as the Integrator decides it, so that a run
    // does not fail at its first transfer.
    const int time_order = fem::Assembler().time_order(*problem);
    if (!options_.damping_method->capabilities().supports(time_order))
      throw std::invalid_argument(std::format(
          "{}: the damping method {} does not support time order {}", stage,
          options_.damping_method->name(), time_order));
  }
  if (target.max_nodes && problem->mesh()->nodes.size() > *target.max_nodes)
    throw std::invalid_argument(std::format(
        "{}: the starting mesh has {} nodes, more than max_nodes = {}", stage,
        problem->mesh()->nodes.size(), *target.max_nodes));

  const auto start = std::chrono::steady_clock::now();
  AdaptiveIntegrationResult result;
  std::optional<double> tolerance;
  const auto &times = output.snapshot_times;
  std::size_t next_snapshot = 0; // into times
  std::size_t steps = 0, adaptation_times = 0, refinements = 0;
  std::size_t damping_steps = 0;
  std::size_t max_nodes_used = 0;
  bool adapting = true;
  std::string stop_message;
  // The state the next piece starts from: the caller's for the first.
  std::optional<solve::Solution> carried;
  const solve::Solution *state_in = initial;
  double t = t0;
  // The stretch to the next adaptation time (or to t1 once adapting has
  // stopped): its end, its step limit and the steps taken in it. Usually
  // one piece; after a transfer with damping steps two (the damping
  // steps, then the integrator's method).
  bool new_stretch = true;
  double end = t1;
  std::optional<std::size_t> max_steps;
  std::size_t stretch_steps = 0;
  // Damping steps still to take since the last transfer.
  std::size_t pending_damping = 0;

  for (;;) {
    // 1. The piece: to the next adaptation time, or to t1 once adapting
    // has stopped.
    if (new_stretch) {
      new_stretch = false;
      end = t1;
      max_steps.reset();
      stretch_steps = 0;
      if (adapting) {
        if (target.every.interval) {
          const double interval = *target.every.interval;
          const double next = t + interval;
          if (!(next > t))
            throw std::runtime_error(std::format(
                "{}: the adaptation interval {} is too small to change t = "
                "{} in double precision",
                stage, interval, t));
          const double slack = landing_tolerance * interval;
          if (next < t1 - slack) {
            end = next;
            // A snapshot time this close is the adaptation time: the piece
            // lands on it exactly, with no step of rounding size after it.
            for (std::size_t s = next_snapshot;
                 s < times.size() && times[s] <= next + slack; ++s)
              if (std::abs(times[s] - next) <= slack) {
                end = times[s];
                break;
              }
          }
        }
        max_steps = target.every.steps;
      }
    }
    // The steps this piece may take: what is left of the stretch's, and
    // at most the damping steps still to take for a damping piece.
    const bool damping = pending_damping > 0;
    std::optional<std::size_t> piece_steps;
    if (max_steps)
      piece_steps = *max_steps - stretch_steps;
    if (damping)
      piece_steps = std::min(
          piece_steps.value_or(std::numeric_limits<std::size_t>::max()),
          pending_damping);
    // The state before the last step is needed only when the piece can end
    // in an adaptation (before t1, or after max_steps steps), and after
    // damping steps for the acceleration of their last step.
    solve::IntegrationOutput piece_output{
        .keep_step_history = output.keep_step_history,
        .keep_before_last_step =
            damping || (adapting && (end < t1 || max_steps))};
    for (std::size_t s = next_snapshot; s < times.size() && times[s] <= end;
         ++s)
      piece_output.snapshot_times.push_back(times[s]);
    const solve::Integrator &stepper = damping ? *damping_ : integrator_;
    solve::IntegrationResult run =
        stepper.integrate(problem, t, end, piece_output, state_in, piece_steps);
    const auto taken = static_cast<std::size_t>(run.report.metrics.at("steps"));
    steps += taken;
    stretch_steps += taken;
    if (damping) {
      damping_steps += taken;
      pending_damping -= std::min(pending_damping, taken);
      // The acceleration of the last damping step, for a method that keeps
      // one (Newmark continues from it without its start).
      if (!run.final.velocities.empty())
        run.final.accelerations =
            last_step_derivatives(*problem, run.final, *run.before_last_step)
                .u_tt;
    }
    // The snapshots taken are the first of the times passed, in order.
    next_snapshot += run.snapshots.size();
    for (auto &s : run.snapshots)
      result.snapshots.push_back(std::move(s));
    for (const auto &record : run.history)
      result.history.push_back(record);
    max_nodes_used = std::max(max_nodes_used, problem->mesh()->nodes.size());
    IntegrationPiece piece{.problem = problem,
                           .t_start = t,
                           .t_end = run.final.time,
                           .report = std::move(run.report),
                           .adaptation = {},
                           .damping = damping};
    t = run.final.time;
    if (!(t < t1)) {
      result.final = std::move(run.final);
      result.pieces.push_back(std::move(piece));
      break;
    }
    // Damping steps that ended before the adaptation time: the
    // integrator's method continues the stretch on the same problem.
    if (t < end && !(max_steps && stretch_steps >= *max_steps)) {
      carried = std::move(run.final);
      state_in = &*carried;
      result.pieces.push_back(std::move(piece));
      continue;
    }
    new_stretch = true;

    // 2. The adaptation time: estimate the solution with its derivatives
    // from the last step.
    ++adaptation_times;
    AdaptationPoint point{.time = t, .steps = steps};
    const solve::Solution &solution = run.final;
    const Derivatives derivatives =
        last_step_derivatives(*problem, solution, *run.before_last_step);
    const SolvedState state{
        .problem = *problem,
        .solution = solution,
        .derivatives = {.u_t = derivatives.u_t, .u_tt = derivatives.u_tt}};
    if (estimating) {
      point.estimate = estimating->estimate(state);
      if (!point.estimate)
        throw std::runtime_error(
            std::format("{}: {} declares estimates_error but gave no estimate",
                        stage, estimating->name()));
    }
    if (has_tolerance && !tolerance)
      tolerance = resolve(target.tolerance, point.estimate->total);

    // 3. Adapt once, unless the tolerance is met.
    std::optional<solve::Solution> moved;
    if (tolerance && point.estimate->total <= *tolerance) {
      point.outcome = AdaptationOutcome::tolerance_met;
    } else {
      std::shared_ptr<const mesh::Mesh> next;
      std::optional<mesh::Parents> parents;
      for (const auto &adaptation : adaptations_) {
        const bool own = adaptation.get() == estimating;
        AdaptationResult adapted =
            adaptation->adapt(state, own ? &*point.estimate : nullptr);
        for (auto &r : adapted.reports)
          point.reports.push_back(std::move(r));
        if (adapted.mesh && !adaptation->capabilities().changes_mesh)
          throw std::runtime_error(std::format(
              "{}: {} returned a mesh but does not declare changes_mesh", stage,
              adaptation->name()));
        if (adapted.mesh) {
          next = std::move(adapted.mesh);
          parents = std::move(adapted.parents);
        }
      }
      if (!next) {
        point.outcome = AdaptationOutcome::nothing_changed;
      } else if (target.max_nodes && next->nodes.size() > *target.max_nodes) {
        point.outcome = AdaptationOutcome::node_budget;
        adapting = false;
        stop_message = std::format(
            "node budget: at t = {} the adapted mesh would have {} nodes, "
            "more than max_nodes = {}; it was discarded, adapting stopped, "
            "and the run continued to t1 on the mesh of {} nodes",
            t, next->nodes.size(), *target.max_nodes,
            problem->mesh()->nodes.size());
      } else {
        auto next_problem = std::make_shared<const fem::MeshedProblem>(
            next, problem->problem());
        point.reports.push_back(next_problem->report());
        SolutionTransfer transfer = transfer_solution(
            solution, *next_problem, parents ? &*parents : nullptr);
        point.reports.push_back(std::move(transfer.report));
        moved = std::move(transfer.solution);
        problem = std::move(next_problem);
        point.outcome = AdaptationOutcome::refined;
        ++refinements;
        pending_damping = options_.damping_steps;
        if (target.max_refinements && refinements >= *target.max_refinements) {
          adapting = false;
          stop_message = std::format(
              "refinement limit: refinement {} of max_refinements = {} was "
              "made at t = {}; adapting stopped, and the run continued to t1 "
              "on the mesh of {} nodes",
              refinements, *target.max_refinements, t,
              problem->mesh()->nodes.size());
        }
      }
    }
    carried = moved ? std::move(*moved) : std::move(run.final);
    state_in = &*carried;
    piece.adaptation = std::move(point);
    result.pieces.push_back(std::move(piece));
  }

  mesh::StageReport &report = result.report;
  report.stage = stage;
  auto &m = report.metrics;
  m["t0"] = t0;
  m["t1"] = t1;
  m["pieces"] = static_cast<double>(result.pieces.size());
  m["adaptation_times"] = static_cast<double>(adaptation_times);
  m["refinements"] = static_cast<double>(refinements);
  m["steps"] = static_cast<double>(steps);
  m["damping_steps"] = static_cast<double>(damping_steps);
  m["nodes"] = static_cast<double>(problem->mesh()->nodes.size());
  m["triangles"] = static_cast<double>(problem->mesh()->triangles.size());
  m["max_nodes_used"] = static_cast<double>(max_nodes_used);
  if (tolerance)
    m["tolerance"] = *tolerance;
  if (!stop_message.empty())
    report.messages.push_back(std::move(stop_message));
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::adapt
