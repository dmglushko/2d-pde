#include "pde2d/adapt/error_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::adapt {

namespace {

void check_values(std::span<const double> values, std::size_t nodes,
                  const std::string &who, const char *what, bool optional) {
  if (optional && values.empty())
    return;
  if (values.size() != nodes)
    throw std::invalid_argument(std::format(
        "{}: {} has {} values for {} mesh nodes (one per node is "
        "needed{})",
        who, what, values.size(), nodes, optional ? ", or none" : ""));
  for (std::size_t i = 0; i < values.size(); ++i)
    if (!std::isfinite(values[i]))
      throw std::invalid_argument(std::format(
          "{}: {} is not finite at node {}: {}", who, what, i, values[i]));
}

} // namespace

ErrorEstimate
ErrorEstimator::estimate(const fem::MeshedProblem &problem,
                         const solve::Solution &solution,
                         const TimeDerivatives &derivatives) const {
  const std::string who = name();
  if (solution.mesh != problem.mesh())
    throw std::invalid_argument(
        who + ": the solution is not on the problem's mesh (the same mesh "
              "object is needed)");
  if (!std::isfinite(solution.time))
    throw std::invalid_argument(who + ": the solution's time is not finite");
  const std::size_t nodes = problem.mesh()->nodes.size();
  check_values(solution.values, nodes, who, "the solution", false);
  check_values(derivatives.u_t, nodes, who, "u_t", true);
  check_values(derivatives.u_tt, nodes, who, "u_tt", true);

  const auto start = std::chrono::steady_clock::now();
  ErrorEstimate out;
  out.report.stage = who;
  out.indicators = indicators(problem, solution, derivatives, out.report);
  const std::size_t triangles = problem.mesh()->triangles.size();
  if (out.indicators.size() != triangles)
    throw std::runtime_error(std::format("{}: {} indicators for {} triangles",
                                         who, out.indicators.size(),
                                         triangles));
  double sum = 0.0, largest = 0.0;
  for (std::size_t t = 0; t < triangles; ++t) {
    const double eta = out.indicators[t];
    if (!std::isfinite(eta) || eta < 0.0)
      throw std::runtime_error(std::format(
          "{}: the indicator of triangle {} is not a finite non-negative "
          "number: {}",
          who, t, eta));
    sum += eta * eta;
    largest = std::max(largest, eta);
  }
  out.total = std::sqrt(sum);
  out.report.metrics["triangles"] = static_cast<double>(triangles);
  out.report.metrics["total"] = out.total;
  out.report.metrics["max_indicator"] = largest;
  out.report.time = std::chrono::steady_clock::now() - start;
  return out;
}

} // namespace pde2d::adapt
