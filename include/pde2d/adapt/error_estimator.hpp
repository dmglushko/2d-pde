#pragma once

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/solution.hpp"

#include <span>
#include <string>
#include <vector>

// Error estimation: the first of the three parts of adaptive refinement
// (estimate -> mark -> refine). An estimator turns a solution into one
// error indicator per triangle.
namespace pde2d::adapt {

// The time derivatives of a solution, one value per mesh node (indexed like
// Mesh::nodes) each, read as P1 functions like the solution. Empty means
// "not given". A static solution (solve::StaticSolver) is a steady state:
// give zeros.
struct TimeDerivatives {
  std::span<const double> u_t{};
  std::span<const double> u_tt{};
};

// What an estimator returns.
struct ErrorEstimate {
  // eta_T, one per triangle (indicators[t] belongs to mesh.triangles[t]),
  // finite and non-negative: what a MarkingStrategy takes.
  std::vector<double> indicators;
  // The estimated error: sqrt(sum over T of eta_T^2).
  double total = 0.0;
  // The estimator's name() as the stage; the time taken; the metrics
  // triangles, total and max_indicator, and the estimator's own parts.
  mesh::StageReport report;
};

// The common interface of the error estimators. Each estimator owns its
// options (if it has any), set with designated initializers in its
// constructor and validated there (std::invalid_argument).
//
// estimate() is not virtual: it checks the input, times the work, adds the
// total and fills the shared metrics, and asks the estimator for the
// indicators (indicators()), so the checks are written once.
class ErrorEstimator {
public:
  virtual ~ErrorEstimator() = default;
  virtual std::string name() const = 0;

  // The error indicators of `solution`, a solution of `problem` at
  // solution.time. The solution must be on the problem's mesh (the same
  // object) with one finite value per node. `derivatives`: u_t and u_tt
  // of the solution, each empty or one finite value per node; which ones
  // an estimator needs is its own rule (the residual estimator: u_t when
  // the problem has a term in u_t, u_tt when it has one in u_tt; the
  // others need none and ignore them). Throws std::invalid_argument for
  // a bad input, naming it, and passes on the estimator's exceptions (a
  // coefficient that is not finite at a quadrature point).
  [[nodiscard]] ErrorEstimate
  estimate(const fem::MeshedProblem &problem, const solve::Solution &solution,
           const TimeDerivatives &derivatives = {}) const;

private:
  // Called by estimate() with a checked input. Returns one indicator per
  // triangle and may add metrics and messages to `report`.
  virtual std::vector<double> indicators(const fem::MeshedProblem &problem,
                                         const solve::Solution &solution,
                                         const TimeDerivatives &derivatives,
                                         mesh::StageReport &report) const = 0;
};

} // namespace pde2d::adapt
