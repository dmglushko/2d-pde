#pragma once

// Private: the prepared solver shared by the iterative methods (BiCgStab,
// ConjugateGradient). They differ only in the Eigen solver; the checks,
// the scaling of the right-hand side, the restarts on the measured
// residual and the error messages are the same. Includes Eigen: only the
// solver .cpp files include this header.

#include "eigen_view.hpp"
#include "vectors.hpp"

#include "pde2d/solve/linear_solver.hpp"

#include <Eigen/IterativeLinearSolvers>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::solve::detail {

// The options every iterative method has.
struct IterativeOptions {
  double tolerance = 1e-10;
  std::size_t max_iterations = 1000;
};

// Throws std::invalid_argument for a tolerance outside (0, 1) or no
// iterations, naming `solver`.
inline void check_options(const IterativeOptions &options,
                          const std::string &solver) {
  if (!(options.tolerance > 0.0 && options.tolerance < 1.0))
    throw std::invalid_argument(solver + ": tolerance must be in (0, 1)");
  if (options.max_iterations < 1)
    throw std::invalid_argument(solver +
                                ": max_iterations must be at least 1");
}

// `EigenSolver` is an Eigen iterative solver for RowMajorMatrix with a
// diagonal preconditioner, for example Eigen::BiCGSTAB<RowMajorMatrix,
// Eigen::DiagonalPreconditioner<double>>.
template <class EigenSolver>
class PreparedIterative final : public PreparedLinearSolver {
public:
  PreparedIterative(std::shared_ptr<const fem::CsrMatrix> matrix,
                    const IterativeOptions &options, std::string name)
      : matrix_(std::move(matrix)), options_(options), name_(std::move(name)) {
    if (matrix_->rows() == 0)
      return;
    // Checks here, once, that the preconditioner can be built. Each solve
    // builds its own (see solve). Eigen keeps a view of the matrix's
    // arrays (no copy); matrix_ keeps them alive.
    EigenSolver solver;
    solver.compute(view(*matrix_));
    if (solver.info() != Eigen::Success)
      throw std::runtime_error(name_ + ": the preconditioner failed");
  }

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    check_right_hand_side(matrix_->rows(), right_hand_side, name_);
    LinearSolveResult result;
    if (matrix_->rows() == 0)
      return result;
    // A zero right-hand side: x = 0 solves it exactly, with no iterations.
    const double scale = max_abs(right_hand_side);
    if (scale == 0.0) {
      result.values.assign(right_hand_side.size(), 0.0);
      return result;
    }
    if (!std::isfinite(scale))
      throw std::invalid_argument(
          name_ + ": the right-hand side has a value that is not finite");

    // Solve A y = b / scale, then x = scale * y. Eigen compares squared
    // norms, which underflow to 0 for a tiny b (1e-200) and would make it
    // return x = 0 at once; scaled, the largest entry of b is 1.
    Eigen::VectorXd b(static_cast<Eigen::Index>(right_hand_side.size()));
    for (std::size_t i = 0; i < right_hand_side.size(); ++i)
      b[static_cast<Eigen::Index>(i)] = right_hand_side[i] / scale;
    const auto matrix = view(*matrix_);

    // Eigen stops on its own estimate of the residual, which can differ
    // from the true one by rounding. The true relative residual is
    // measured after each run; above the tolerance, the solver restarts
    // from its answer until the iteration budget is spent or a restart no
    // longer halves the residual (the tolerance is then below what double
    // precision can reach for this matrix, or the method does not
    // converge for it).
    Eigen::VectorXd y = Eigen::VectorXd::Zero(b.size());
    double previous_residual = std::numeric_limits<double>::infinity();
    std::size_t iterations = 0;
    bool converged = false;
    bool finite = true;
    // A local Eigen solver per solve: its iteration limit changes between
    // runs, and a prepared solver is const and may be used by several
    // threads at once. Computing the diagonal preconditioner costs less
    // than one iteration.
    EigenSolver solver;
    solver.setTolerance(options_.tolerance);
    solver.compute(matrix);
    while (iterations < options_.max_iterations) {
      solver.setMaxIterations(
          static_cast<Eigen::Index>(options_.max_iterations - iterations));
      y = solver.solveWithGuess(b, y);
      // At least one counted iteration per run, so the loop always ends.
      iterations += std::max<std::size_t>(
          1, static_cast<std::size_t>(solver.iterations()));
      if (!y.allFinite()) {
        finite = false;
        break;
      }
      const Eigen::VectorXd r = b - matrix * y;
      // b is scaled so that its largest entry is 1: its norm is >= 1 and
      // the squares below do not underflow for the parts that matter.
      result.residual = r.stableNorm() / b.stableNorm();
      if (solver.info() == Eigen::Success &&
          result.residual <= options_.tolerance) {
        converged = true;
        break;
      }
      if (solver.info() != Eigen::Success)
        break; // Eigen ran out of iterations or broke down
      if (!(result.residual < 0.5 * previous_residual))
        break; // restarting no longer helps
      previous_residual = result.residual;
    }
    result.iterations = iterations;
    result.values.resize(right_hand_side.size());
    for (std::size_t i = 0; i < right_hand_side.size(); ++i)
      result.values[i] = scale * y[static_cast<Eigen::Index>(i)];
    finite = finite && all_finite(result.values);
    if (!converged || !finite) {
      std::ostringstream s;
      s << name_ << ": did not converge in " << result.iterations
        << " iterations (maximum " << options_.max_iterations
        << ", relative residual " << (finite ? result.residual : std::nan(""))
        << ", tolerance " << options_.tolerance << ")";
      throw std::runtime_error(s.str());
    }
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  IterativeOptions options_;
  std::string name_;
};

} // namespace pde2d::solve::detail
