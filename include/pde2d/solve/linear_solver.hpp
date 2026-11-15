#pragma once

#include "pde2d/fem/csr_matrix.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

// The linear solver interface. Solving has two steps, like Eigen's
// compute() and solve(): prepare(matrix) does the work that depends only
// on the matrix (a factorization, a preconditioner) and returns a prepared
// solver tied to that matrix, which then solves for as many right-hand
// sides as needed. A configured LinearSolver is const and can prepare
// many matrices.
//
// A solver declares what it needs of a matrix (requirements()); the
// Integrator and the StaticSolver check that before they prepare a matrix,
// against the facts assembly gives each operator (fem::Operator::
// always_symmetric, symmetric_now) or, for a step matrix the facts cannot
// promise, against its values (fem::is_symmetric). What is not checked
// (positive definiteness) is not declared: a solver that needs it detects
// its failure and throws std::runtime_error.
namespace pde2d::solve {

// What a solver needs of the matrices it prepares.
struct LinearSolverRequirements {
  // The matrix must be symmetric (it may differ from its transpose only by
  // rounding).
  bool symmetric_matrix = false;
};

// The result of one solve.
struct LinearSolveResult {
  // The solution x of matrix * x = right-hand side, one value per row
  // (for the assembled systems: per free unknown).
  std::vector<double> values{};
  // Iterations of an iterative solver; 0 for a direct one.
  std::size_t iterations = 0;
  // The relative residual |b - A x| / |b| in the Euclidean norm, computed
  // from x after the solve (|b - A x| when b is zero).
  double residual = 0.0;
};

// A solver prepared for one matrix. It holds the matrix (shared), so the
// matrix stays alive and unchanged as long as the prepared solver does.
class PreparedLinearSolver {
public:
  virtual ~PreparedLinearSolver() = default;
  // Solves matrix * x = right_hand_side. Throws std::invalid_argument when
  // the size of right_hand_side is not the number of rows, and
  // std::runtime_error when the solve fails (an iterative solver that does
  // not converge, a result that is not finite).
  virtual LinearSolveResult
  solve(std::span<const double> right_hand_side) const = 0;
};

class LinearSolver {
public:
  virtual ~LinearSolver() = default;
  // Used in error messages and reports.
  virtual std::string name() const = 0;
  // What the matrices given to prepare() must satisfy.
  virtual LinearSolverRequirements requirements() const = 0;
  // Prepares a solver for `matrix`. Throws std::invalid_argument for a
  // null matrix, one without a pattern or one that is not square, and
  // std::runtime_error when the preparation fails (a singular matrix for
  // a direct solver).
  virtual std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const = 0;
};

} // namespace pde2d::solve
