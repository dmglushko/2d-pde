#include "pde2d/solve/static_solver.hpp"

#include "pde2d/fem/assembler.hpp"

#include "vectors.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pde2d::solve {

StaticSolver::StaticSolver(std::shared_ptr<const LinearSolver> linear_solver)
    : linear_solver_(std::move(linear_solver)) {
  if (!linear_solver_)
    throw std::invalid_argument("static_solver: the linear solver is null");
}

StaticResult
StaticSolver::solve(std::shared_ptr<const fem::MeshedProblem> meshed_problem,
                    double t) const {
  if (!meshed_problem)
    throw std::invalid_argument("static_solver: the problem is null");
  if (!std::isfinite(t))
    throw std::invalid_argument("static_solver: t is not finite");

  const fem::AssembledProblem assembled =
      fem::Assembler().assemble(meshed_problem, t);
  const auto start = std::chrono::steady_clock::now();

  // b = load - A_FD U_D. Not right_hand_side(): for a problem of time
  // order 1 or 2 that also holds the lifting of U_D' and U_D'', which
  // belong to the time terms that a static solve drops.
  std::vector<double> rhs(assembled.load().begin(), assembled.load().end());
  const fem::Operator &stiffness = assembled.stiffness();
  if (!stiffness.is_zero && !assembled.dirichlet_values().empty())
    detail::add_scaled(rhs, -1.0,
                       stiffness.dirichlet.multiply(
                           assembled.dirichlet_values()));

  const auto prepared = linear_solver_->prepare(
      std::make_shared<const fem::CsrMatrix>(stiffness.free));
  const LinearSolveResult solved = prepared->solve(rhs);

  StaticResult result;
  result.solution = build_solution(*meshed_problem, solved.values, t);
  result.reports.push_back(assembled.report());
  mesh::StageReport report;
  report.stage = "static_solver";
  report.metrics["free_unknowns"] =
      static_cast<double>(meshed_problem->free_nodes().size());
  report.metrics["linear_iterations"] =
      static_cast<double>(solved.iterations);
  report.metrics["residual"] = solved.residual;
  report.time = std::chrono::steady_clock::now() - start;
  result.reports.push_back(std::move(report));
  return result;
}

} // namespace pde2d::solve
