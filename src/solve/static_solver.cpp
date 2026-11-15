#include "pde2d/solve/static_solver.hpp"

#include "pde2d/fem/assembler.hpp"

#include "requirements.hpp"
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

  // Time order 0: only A and the load are built, and only U_D is
  // evaluated, so right_hand_side() is load - A_FD U_D and no derivative of
  // the boundary data is needed (d/dt sqrt(t) at t = 0, issue #35).
  const fem::AssembledProblem assembled = fem::Assembler().assemble(
      meshed_problem, t, nullptr, /*max_time_order=*/0);
  const auto start = std::chrono::steady_clock::now();

  const fem::Operator &stiffness = assembled.stiffness();
  // Symmetric by the facts, or else by the values at t (symmetric_now
  // already includes the facts; both are named to show the rule).
  detail::check_requirements(
      *linear_solver_, stiffness.always_symmetric || stiffness.symmetric_now,
      "static_solver", "A (the stiffness)");
  const auto prepared = linear_solver_->prepare(
      std::make_shared<const fem::CsrMatrix>(stiffness.free));
  const LinearSolveResult solved = prepared->solve(assembled.right_hand_side());

  StaticResult result;
  result.solution = build_solution(*meshed_problem, solved.values, t);
  result.reports.push_back(assembled.report());
  mesh::StageReport report;
  report.stage = "static_solver";
  report.metrics["free_unknowns"] =
      static_cast<double>(meshed_problem->free_nodes().size());
  report.metrics["linear_iterations"] = static_cast<double>(solved.iterations);
  report.metrics["residual"] = solved.residual;
  report.time = std::chrono::steady_clock::now() - start;
  result.reports.push_back(std::move(report));
  return result;
}

} // namespace pde2d::solve
