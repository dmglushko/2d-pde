#pragma once

// Private: checking a linear solver's declared requirements against what
// assembly's facts say about a matrix, before it is prepared. prepare()
// itself sees only the values, so the check lives with the callers (the
// Integrator, the StaticSolver and Newmark's start).

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/time_method.hpp"

#include <stdexcept>
#include <string>

namespace pde2d::solve::detail {

// Throws std::invalid_argument naming `caller`, the solver, `matrix` and
// the reason when the solver requires a symmetric matrix and `symmetric`
// says the matrix is not.
inline void check_requirements(const LinearSolver &solver, bool symmetric,
                               const std::string &caller,
                               const std::string &matrix) {
  if (solver.requirements().symmetric_matrix && !symmetric)
    throw std::invalid_argument(
        caller + ": the linear solver " + solver.name() +
        " requires a symmetric matrix, but " + matrix +
        " is not symmetric by assembly's facts (a convection term, or a "
        "tangential boundary term from an anisotropic A or from beta_tau, "
        "beta_x, beta_y); use a solver for any matrix, such as sparse_lu or "
        "bicgstab");
}

// Whether weights.mass M2 + weights.damping B + weights.stiffness A is
// symmetric by the facts: every operator with a non-zero weight is
// symmetric or zero.
inline bool step_matrix_is_symmetric(const fem::AssembledProblem &problem,
                                     const StepMatrixWeights &weights) {
  const auto fine = [](const fem::Operator &op, double weight) {
    return weight == 0.0 || op.is_zero || op.symmetric;
  };
  return fine(problem.mass(), weights.mass) &&
         fine(problem.damping(), weights.damping) &&
         fine(problem.stiffness(), weights.stiffness);
}

} // namespace pde2d::solve::detail
