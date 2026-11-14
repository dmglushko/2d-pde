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
// problem at t and solves
//   A U = load - A_FD U_D(t)
// (stiffness().free, the load, the stiffness's Dirichlet part and the
// Dirichlet values). The lifting terms of U_D' and U_D'' are left out with
// the time terms. Any time order is accepted: the time terms are dropped.
class StaticSolver {
public:
  // Throws std::invalid_argument for a null linear solver.
  explicit StaticSolver(std::shared_ptr<const LinearSolver> linear_solver);

  // Throws std::invalid_argument for a null problem or a t that is not
  // finite, and passes on the exceptions of assembly and of the linear
  // solver (a singular matrix throws std::runtime_error).
  [[nodiscard]] StaticResult
  solve(std::shared_ptr<const fem::MeshedProblem> meshed_problem,
        double t = 0.0) const;

private:
  std::shared_ptr<const LinearSolver> linear_solver_;
};

} // namespace pde2d::solve
