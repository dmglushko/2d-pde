#pragma once

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/solution.hpp"

#include <memory>
#include <vector>

namespace pde2d::solve {

struct StaticResult {
  Solution solution{};
  // "assembly" (the Assembler's report), then "static_solver": the time
  // taken and the metrics free_unknowns, linear_iterations and residual.
  std::vector<mesh::StageReport> reports{};
};

// Solves a problem without its time terms at one time t: it assembles the
// problem at t with max_time_order 0 (only A and the load; only U_D is
// evaluated, never its time derivatives) and solves
//   A U = load - A_FD U_D(t)
// (stiffness().free and right_hand_side()). Any time order is accepted:
// the time terms are dropped. The linear solver's requirements are
// checked against the facts of A first.
class StaticSolver {
public:
  // Throws std::invalid_argument for a null linear solver.
  explicit StaticSolver(std::shared_ptr<const LinearSolver> linear_solver);

  // Throws std::invalid_argument for a null problem, a t that is not
  // finite, or a linear solver that requires a symmetric matrix when A is
  // not symmetric (fem::Operator::symmetric), and passes on the
  // exceptions of assembly and of the linear solver (a singular matrix
  // throws std::runtime_error).
  [[nodiscard]] StaticResult
  solve(std::shared_ptr<const fem::MeshedProblem> meshed_problem,
        double t = 0.0) const;

private:
  std::shared_ptr<const LinearSolver> linear_solver_;
};

} // namespace pde2d::solve
