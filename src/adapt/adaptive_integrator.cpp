#include "pde2d/adapt/adaptive_integrator.hpp"

#include "pde2d/adapt/solution_transfer.hpp"

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

// An adaptation time within this fraction of the interval of t1 is t1, so
// that rounding in the sums t_k + interval never leaves a tiny last piece
// (the Integrator's landing rule for its steps).
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

// The time derivatives at the end of a piece, at every node, from its last
// step: u_t = (u(t) - u(t - dt)) / dt, or for time order 2 (velocities)
// u_t = the velocities and u_tt = their difference quotient.
struct Derivatives {
  std::vector<double> u_t, u_tt;
};

Derivatives last_step_derivatives(const solve::Solution &end,
                                  const solve::Solution &before) {
  const double dt = end.time - before.time;
  const auto quotient = [dt](const std::vector<double> &now,
                             const std::vector<double> &then) {
    std::vector<double> rate(now.size());
    for (std::size_t i = 0; i < now.size(); ++i)
      rate[i] = (now[i] - then[i]) / dt;
    return rate;
  };
  Derivatives d;
  if (!end.velocities.empty()) {
    d.u_t = end.velocities;
    d.u_tt = quotient(end.velocities, before.velocities);
  } else {
    d.u_t = quotient(end.values, before.values);
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
    std::vector<std::shared_ptr<const Adaptation>> adaptations)
    : integrator_(std::move(integrator)), adaptations_(std::move(adaptations)) {
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
  std::size_t max_nodes_used = 0;
  bool adapting = true;
  std::string budget_message;
  // The state the next piece starts from: the caller's for the first.
  std::optional<solve::Solution> carried;
  const solve::Solution *state_in = initial;
  double t = t0;

  for (;;) {
    // 1. The piece: to the next adaptation time, or to t1 once adapting
    // has stopped.
    double end = t1;
    std::optional<std::size_t> max_steps;
    if (adapting) {
      if (target.every.interval) {
        const double next = t + *target.every.interval;
        if (next < t1 - landing_tolerance * *target.every.interval)
          end = next;
      }
      max_steps = target.every.steps;
    }
    solve::IntegrationOutput piece_output{.keep_step_history =
                                              output.keep_step_history,
                                          .keep_before_last_step = adapting};
    for (std::size_t s = next_snapshot; s < times.size() && times[s] <= end;
         ++s)
      piece_output.snapshot_times.push_back(times[s]);
    solve::IntegrationResult run = integrator_.integrate(
        problem, t, end, piece_output, state_in, max_steps);
    steps += static_cast<std::size_t>(run.report.metrics.at("steps"));
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
                           .adaptation = {}};
    t = run.final.time;
    if (!(t < t1)) {
      result.final = std::move(run.final);
      result.pieces.push_back(std::move(piece));
      break;
    }

    // 2. The adaptation time: estimate the solution with its derivatives
    // from the last step.
    ++adaptation_times;
    AdaptationPoint point{.time = t, .steps = steps};
    const solve::Solution &solution = run.final;
    const Derivatives derivatives =
        last_step_derivatives(solution, *run.before_last_step);
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
        budget_message = std::format(
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
  m["nodes"] = static_cast<double>(problem->mesh()->nodes.size());
  m["triangles"] = static_cast<double>(problem->mesh()->triangles.size());
  m["max_nodes_used"] = static_cast<double>(max_nodes_used);
  if (tolerance)
    m["tolerance"] = *tolerance;
  if (!budget_message.empty())
    report.messages.push_back(std::move(budget_message));
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::adapt
