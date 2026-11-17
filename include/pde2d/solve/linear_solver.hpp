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
  // The normwise backward error |b - A x| / (|A| |x| + |b|) in the
  // infinity norm (|A| the largest absolute row sum), from the same
  // residual: the smallest relative change of A and b for which x is the
  // exact solution. Unlike `residual`, it is small (a few machine
  // epsilons) for every accurate direct solve, however large the
  // coefficient contrast or the condition number; the direct solvers
  // SimplicialLdlt, SupernodalCholesky and MultifrontalLu check it against
  // their residual_tolerance. 0 for b = 0 and x = 0.
  double backward_error = 0.0;
  // The ratio of the smallest pivot to the largest (in magnitude) of the
  // factorization that solved, where the solver measures it:
  // SimplicialLdlt and SupernodalCholesky (the diagonal D of L D L^T, for
  // the Cholesky the squares of the diagonal of L), MultifrontalLu (the
  // diagonal of U of the row-scaled matrix). Reported, not judged: a tiny
  // ratio (1e-17 for a pure Neumann problem) says the matrix is singular
  // or nearly so, but these solvers refuse only an exact zero pivot. 0
  // when not measured (the iterative solvers, SparseLu, an empty system);
  // a measured ratio is above 0.
  double pivot_ratio = 0.0;
  // The threads the solve could use: the resolved `threads` option of the
  // iterative solvers (for their matrix-vector products) and of the
  // parallel direct solvers (SupernodalCholesky, MultifrontalLu); 1 for
  // the Eigen direct solvers (SparseLu, SimplicialLdlt), which run in one
  // thread, and for an empty system.
  std::size_t threads = 1;
  // Warnings: the solve worked, but the caller should know something
  // (failures are exceptions). Each names the solver. Empty for most
  // solves; SupernodalCholesky gives one on every solve with a matrix it
  // prepared as negative definite (it factored -A). The StaticSolver puts
  // them in its "static_solver" report, the Integrator each distinct one
  // once in its "integrator" report.
  std::vector<std::string> messages{};
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
