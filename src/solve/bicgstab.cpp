#include "pde2d/solve/bicgstab.hpp"

#include "eigen_view.hpp"
#include "vectors.hpp"

#include <Eigen/IterativeLinearSolvers>

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace pde2d::solve {

namespace {

using Solver = Eigen::BiCGSTAB<detail::RowMajorMatrix,
                               Eigen::DiagonalPreconditioner<double>>;

class PreparedBiCgStab final : public PreparedLinearSolver {
public:
  PreparedBiCgStab(std::shared_ptr<const fem::CsrMatrix> matrix,
                   const BiCgStabOptions &options)
      : matrix_(std::move(matrix)), options_(options) {
    if (matrix_->rows() == 0)
      return;
    // Checks here, once, that the preconditioner can be built. Each solve
    // builds its own (see solve). Eigen keeps a view of the matrix's
    // arrays (no copy); matrix_ keeps them alive.
    Solver solver;
    solver.compute(detail::view(*matrix_));
    if (solver.info() != Eigen::Success)
      throw std::runtime_error("bicgstab: the preconditioner failed");
  }

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    detail::check_right_hand_side(matrix_->rows(), right_hand_side, "bicgstab");
    LinearSolveResult result;
    if (matrix_->rows() == 0)
      return result;
    // A zero right-hand side: x = 0 solves it exactly, with no iterations.
    const double scale = detail::max_abs(right_hand_side);
    if (scale == 0.0) {
      result.values.assign(right_hand_side.size(), 0.0);
      return result;
    }
    if (!std::isfinite(scale))
      throw std::invalid_argument(
          "bicgstab: the right-hand side has a value that is not finite");

    // Solve A y = b / scale, then x = scale * y. Eigen compares squared
    // norms, which underflow to 0 for a tiny b (1e-200) and would make it
    // return x = 0 at once; scaled, the largest entry of b is 1.
    Eigen::VectorXd b(static_cast<Eigen::Index>(right_hand_side.size()));
    for (std::size_t i = 0; i < right_hand_side.size(); ++i)
      b[static_cast<Eigen::Index>(i)] = right_hand_side[i] / scale;
    const auto matrix = detail::view(*matrix_);

    // Eigen stops on its own estimate of the residual, which can differ
    // from the true one by rounding. The true relative residual is
    // measured after each run; above the tolerance, the solver restarts
    // from its answer until the iteration budget is spent or a restart no
    // longer halves the residual (the tolerance is then below what double
    // precision can reach for this matrix).
    Eigen::VectorXd y = Eigen::VectorXd::Zero(b.size());
    double previous_residual = std::numeric_limits<double>::infinity();
    std::size_t iterations = 0;
    bool converged = false;
    bool finite = true;
    // A local Eigen solver per solve: its iteration limit changes between
    // runs, and a prepared solver is const and may be used by several
    // threads at once. Computing the diagonal preconditioner costs less
    // than one iteration.
    Solver solver;
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
    finite = finite && detail::all_finite(result.values);
    if (!converged || !finite) {
      std::ostringstream s;
      s << "bicgstab: did not converge in " << result.iterations
        << " iterations (maximum " << options_.max_iterations
        << ", relative residual " << (finite ? result.residual : std::nan(""))
        << ", tolerance " << options_.tolerance << ")";
      throw std::runtime_error(s.str());
    }
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  BiCgStabOptions options_;
};

} // namespace

BiCgStab::BiCgStab(BiCgStabOptions options) : options_(options) {
  if (!(options_.tolerance > 0.0 && options_.tolerance < 1.0))
    throw std::invalid_argument("bicgstab: tolerance must be in (0, 1)");
  if (options_.max_iterations < 1)
    throw std::invalid_argument("bicgstab: max_iterations must be at least 1");
}

std::unique_ptr<const PreparedLinearSolver>
BiCgStab::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedBiCgStab>(std::move(matrix), options_);
}

} // namespace pde2d::solve
