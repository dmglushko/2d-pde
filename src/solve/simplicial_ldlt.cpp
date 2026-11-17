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
    // The Eigen direct solvers run in one thread (the parallel ones are
    // SupernodalCholesky and MultifrontalLu).
    const detail::EigenThreads scope(1);
    const ColumnMajorMatrix a = detail::view(*matrix_);
    ldlt_.compute(a);
    if (ldlt_.info() != Eigen::Success)
      throw std::runtime_error(
          "simplicial_ldlt: the factorization failed (a zero pivot: the "
          "matrix is singular, or indefinite and needs pivoting)");
    // The pivots are the diagonal D; none is zero after a success.
    const Eigen::VectorXd d = ldlt_.vectorD().cwiseAbs();
    pivot_ratio_ = d.minCoeff() / d.maxCoeff();
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
    const auto measures =
        detail::measure_residual(*matrix_, right_hand_side, result.values);
    result.residual = measures.relative_residual;
    result.backward_error = measures.backward_error;
    result.pivot_ratio = pivot_ratio_;
    // The check that makes LDL^T without pivoting safe: a tiny pivot of an
    // indefinite matrix factorizes "successfully" and gives a wrong x,
    // with a large backward error (the growth of the factors). The
    // relative residual is not checked: for an accurate x it is about
    // machine epsilon times |A| |x| / |b|, which a large coefficient
    // contrast makes large.
    if (!(result.backward_error <= options_.residual_tolerance))
      throw std::runtime_error(std::format(
          "simplicial_ldlt: the backward error {} is above the tolerance {} "
          "(residual_tolerance; the matrix is probably indefinite and needs "
          "pivoting; use sparse_lu or multifrontal_lu)",
          result.backward_error, options_.residual_tolerance));
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  SimplicialLdltOptions options_;
  Ldlt ldlt_;
  double pivot_ratio_ = 0.0; // min |D| / max |D|; 0 for an empty matrix
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
