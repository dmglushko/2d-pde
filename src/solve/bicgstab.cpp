#include "pde2d/solve/bicgstab.hpp"

#include "eigen_view.hpp"
#include "vectors.hpp"

#include <Eigen/IterativeLinearSolvers>

#include <cmath>
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
    solver_.setTolerance(options_.tolerance);
    solver_.setMaxIterations(
        static_cast<Eigen::Index>(options_.max_iterations));
    // The solver keeps a view of the matrix's arrays (no copy); matrix_
    // keeps them alive. The diagonal preconditioner is computed here.
    solver_.compute(detail::view(*matrix_));
    if (solver_.info() != Eigen::Success)
      throw std::runtime_error("bicgstab: the preconditioner failed");
  }

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    detail::check_right_hand_side(matrix_->rows(), right_hand_side, "bicgstab");
    LinearSolveResult result;
    if (matrix_->rows() == 0)
      return result;
    const Eigen::VectorXd x = solver_.solve(detail::view(right_hand_side));
    result.iterations = static_cast<std::size_t>(solver_.iterations());
    result.values.assign(x.data(), x.data() + x.size());
    const bool finite = detail::all_finite(result.values);
    if (finite)
      result.residual =
          detail::relative_residual(*matrix_, right_hand_side, result.values);
    if (solver_.info() != Eigen::Success || !finite) {
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
  Solver solver_;
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
