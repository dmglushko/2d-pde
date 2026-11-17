#pragma once

#include "pde2d/fem/assembler.hpp"
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
  // taken and the metrics free_unknowns, linear_iterations, residual,
  // backward_error, pivot_ratio (0 when the linear solver does not measure
  // it) and linear_threads (LinearSolveResult::threads).
  std::vector<mesh::StageReport> reports{};
};

// Solves a problem without its time terms at one time t: it assembles the
// problem at t with max_time_order 0 (only A and the load; only U_D is
// evaluated, never its time derivatives) and solves
//   A U = load - A_FD U_D(t)
// (stiffness().free and right_hand_side()). Any time order is accepted:
// the time terms are dropped. The linear solver's requirements are
// checked first: A meets a symmetry requirement when it is always
// symmetric by assembly's facts or symmetric at t by its values
// (fem::Operator::always_symmetric, symmetric_now).
class StaticSolver {
public:
  // The assembler is made here with `assembler_options` (its thread
  // count). Throws std::invalid_argument for a null linear solver and
  // assembler options out of range.
  explicit StaticSolver(std::shared_ptr<const LinearSolver> linear_solver,
                        fem::AssemblerOptions assembler_options = {});

  // Throws std::invalid_argument for a null problem, a t that is not
  // finite, or a linear solver that requires a symmetric matrix when A is
  // neither always symmetric nor symmetric at t, and passes on the
  // exceptions of assembly and of the linear solver (a singular matrix
  // throws std::runtime_error).
  [[nodiscard]] StaticResult
  solve(std::shared_ptr<const fem::MeshedProblem> meshed_problem,
        double t = 0.0) const;

private:
  std::shared_ptr<const LinearSolver> linear_solver_;
  fem::Assembler assembler_;
};

} // namespace pde2d::solve
