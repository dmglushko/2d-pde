#include "pde2d/solve/simplicial_ldlt.hpp"

#include "eigen_threads.hpp"
#include "eigen_view.hpp"
#include "vectors.hpp"

#include <Eigen/OrderingMethods>
#include <Eigen/SparseCholesky>

#include <format>
#include <memory>
#include <stdexcept>
#include <utility>

namespace pde2d::solve {

namespace {

// SimplicialLDLT works on a column-major matrix and reads one triangle
// (Lower); the matrix is copied once into that format, as for SparseLu.
using ColumnMajorMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using Ldlt = Eigen::SimplicialLDLT<ColumnMajorMatrix, Eigen::Lower,
                                   Eigen::AMDOrdering<int>>;

class PreparedSimplicialLdlt final : public PreparedLinearSolver {
public:
  PreparedSimplicialLdlt(std::shared_ptr<const fem::CsrMatrix> matrix,
                         const SimplicialLdltOptions &options)
      : matrix_(std::move(matrix)), options_(options) {
    if (matrix_->rows() == 0)
      return; // nothing to factorize: every solve returns an empty vector
    // Direct solvers run in one thread (docs/decisions.md: their parallel
    // version needs a library still to be chosen).
    const detail::EigenThreads scope(1);
    const ColumnMajorMatrix a = detail::view(*matrix_);
    ldlt_.compute(a);
    if (ldlt_.info() != Eigen::Success)
      throw std::runtime_error(
          "simplicial_ldlt: the factorization failed (a zero pivot: the "
          "matrix is singular, or indefinite and needs pivoting)");
  }

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    detail::check_right_hand_side(matrix_->rows(), right_hand_side,
                                  "simplicial_ldlt");
    LinearSolveResult result;
    if (matrix_->rows() == 0)
      return result;
    const detail::EigenThreads scope(1);
    const Eigen::VectorXd x = ldlt_.solve(detail::view(right_hand_side));
    if (ldlt_.info() != Eigen::Success)
      throw std::runtime_error("simplicial_ldlt: the solve failed");
    result.values.assign(x.data(), x.data() + x.size());
    if (!detail::all_finite(result.values))
      throw std::runtime_error(
          "simplicial_ldlt: the solution is not finite (the matrix is "
          "singular or nearly so)");
    result.residual =
        detail::relative_residual(*matrix_, right_hand_side, result.values);
    // The check that makes LDL^T without pivoting safe: a tiny pivot of an
    // indefinite matrix factorizes "successfully" and gives a wrong x.
    if (!(result.residual <= options_.residual_tolerance))
      throw std::runtime_error(std::format(
          "simplicial_ldlt: the relative residual {} is above the tolerance "
          "{} (the matrix is probably indefinite and needs pivoting; use "
          "sparse_lu)",
          result.residual, options_.residual_tolerance));
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  SimplicialLdltOptions options_;
  Ldlt ldlt_;
};

} // namespace

SimplicialLdlt::SimplicialLdlt(SimplicialLdltOptions options)
    : options_(options) {
  if (!(options_.residual_tolerance > 0.0 && options_.residual_tolerance < 1.0))
    throw std::invalid_argument(
        "simplicial_ldlt: residual_tolerance must be in (0, 1)");
}

std::unique_ptr<const PreparedLinearSolver>
SimplicialLdlt::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedSimplicialLdlt>(std::move(matrix),
                                                        options_);
}

} // namespace pde2d::solve
