#include "pde2d/adapt/adaptive_static_solver.hpp"

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

constexpr const char *stage = "adaptive_static_solver";

void check_positive(const std::optional<double> &value, const char *name) {
  if (value && !(std::isfinite(*value) && *value > 0.0))
    throw std::invalid_argument(std::format(
        "{}: {} must be finite and positive, got {}", stage, name, *value));
}

void validate(const AdaptiveTarget &target) {
  const bool tolerance = target.tolerance.absolute || target.tolerance.relative;
  if (!tolerance && !target.max_cycles && !target.max_nodes)
    throw std::invalid_argument(
        std::format("{}: the target needs a tolerance, max_cycles or "
                    "max_nodes",
                    stage));
  check_positive(target.tolerance.absolute, "tolerance.absolute");
  check_positive(target.tolerance.relative, "tolerance.relative");
  if (target.max_cycles && *target.max_cycles == 0)
    throw std::invalid_argument(
        std::format("{}: max_cycles must be at least 1", stage));
  if (target.max_nodes && *target.max_nodes == 0)
    throw std::invalid_argument(
        std::format("{}: max_nodes must be at least 1", stage));
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

} // namespace

std::string_view to_string(StopReason reason) {
  switch (reason) {
  case StopReason::tolerance_met:
    return "tolerance_met";
  case StopReason::max_cycles:
    return "max_cycles";
  case StopReason::node_budget:
    return "node_budget";
  case StopReason::nothing_changed:
    return "nothing_changed";
  }
  return "unknown";
}

AdaptiveStaticSolver::AdaptiveStaticSolver(
    solve::StaticSolver solver,
    std::vector<std::shared_ptr<const Adaptation>> adaptations)
    : solver_(std::move(solver)), adaptations_(std::move(adaptations)) {
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
        "{}: at most one adaptation may change the mesh (until a solution "
        "can be transferred between meshes, all act on the same solved "
        "state); {} and {} both do",
        stage, changing[0], changing[1]));
}

AdaptiveStaticResult
AdaptiveStaticSolver::solve(std::shared_ptr<const fem::MeshedProblem> problem,
                            const AdaptiveTarget &target, double t) const {
  if (!problem)
    throw std::invalid_argument(std::format("{}: the problem is null", stage));
  if (!std::isfinite(t))
    throw std::invalid_argument(std::format("{}: t is not finite", stage));
  validate(target);
  // The adaptation whose estimate the target is tested with.
  const Adaptation *estimating = nullptr;
  for (const auto &adaptation : adaptations_)
    if (!estimating && adaptation->capabilities().estimates_error)
      estimating = adaptation.get();
  const bool has_tolerance =
      target.tolerance.absolute || target.tolerance.relative;
  if (has_tolerance && !estimating)
    throw std::invalid_argument(std::format(
        "{}: a tolerance needs an adaptation that estimates the error",
        stage));
  if (target.max_nodes && problem->mesh()->nodes.size() > *target.max_nodes)
    throw std::invalid_argument(std::format(
        "{}: the starting mesh has {} nodes, more than max_nodes = {}", stage,
        problem->mesh()->nodes.size(), *target.max_nodes));

  const auto start = std::chrono::steady_clock::now();
  AdaptiveStaticResult result;
  std::optional<double> tolerance;
  std::optional<mesh::StageReport> problem_report;
  std::string why;
  for (std::size_t cycle = 1;; ++cycle) {
    AdaptiveCycle current;
    if (problem_report) {
      current.reports.push_back(std::move(*problem_report));
      problem_report.reset();
    }
    solve::StaticResult solved = solver_.solve(problem, t);
    current.solution = std::move(solved.solution);
    for (auto &r : solved.reports)
      current.reports.push_back(std::move(r));
    // A static solution is a steady state: u_t = u_tt = 0.
    const std::vector<double> zeros(problem->mesh()->nodes.size(), 0.0);
    const SolvedState state{.problem = *problem,
                            .solution = current.solution,
                            .derivatives = {.u_t = zeros, .u_tt = zeros}};
    if (estimating)
      current.estimate = estimating->estimate(state);
    if (has_tolerance && !tolerance)
      tolerance = resolve(target.tolerance, current.estimate->total);

    std::optional<StopReason> stop;
    if (tolerance && current.estimate->total <= *tolerance) {
      stop = StopReason::tolerance_met;
      why = std::format("the estimate {} is at most the tolerance {}",
                        current.estimate->total, *tolerance);
    } else if (target.max_cycles && cycle >= *target.max_cycles) {
      stop = StopReason::max_cycles;
      why = std::format("cycle {} of at most {}", cycle, *target.max_cycles);
    }
    if (!stop) {
      std::shared_ptr<const mesh::Mesh> next;
      for (const auto &adaptation : adaptations_) {
        const bool own = adaptation.get() == estimating;
        AdaptationResult adapted = adaptation->adapt(
            state, own ? &*current.estimate : nullptr);
        for (auto &r : adapted.reports)
          current.reports.push_back(std::move(r));
        if (adapted.mesh)
          next = std::move(adapted.mesh);
      }
      if (!next) {
        stop = StopReason::nothing_changed;
        why = "no adaptation changed the mesh";
      } else if (target.max_nodes && next->nodes.size() > *target.max_nodes) {
        stop = StopReason::node_budget;
        why = std::format("the adapted mesh would have {} nodes, more than "
                          "max_nodes = {}; it was not solved",
                          next->nodes.size(), *target.max_nodes);
      } else {
        auto next_problem =
            std::make_shared<const fem::MeshedProblem>(next, problem->problem());
        problem_report = next_problem->report();
        problem = std::move(next_problem);
      }
    }
    result.cycles.push_back(std::move(current));
    if (stop) {
      result.stop_reason = *stop;
      break;
    }
  }

  result.problem = problem;
  mesh::StageReport &report = result.report;
  report.stage = stage;
  report.metrics["cycles"] = static_cast<double>(result.cycles.size());
  report.metrics["nodes"] = static_cast<double>(problem->mesh()->nodes.size());
  report.metrics["triangles"] =
      static_cast<double>(problem->mesh()->triangles.size());
  if (result.cycles.back().estimate)
    report.metrics["estimate"] = result.cycles.back().estimate->total;
  if (tolerance)
    report.metrics["tolerance"] = *tolerance;
  report.messages.push_back(
      std::format("stopped: {}: {}", to_string(result.stop_reason), why));
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::adapt
