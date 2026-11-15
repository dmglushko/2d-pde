#include "pde2d/solve/simplicial_ldlt.hpp"

#include "eigen_view.hpp"
#include "vectors.hpp"

#include <Eigen/OrderingMethods>
#include <Eigen/SparseCholesky>

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
  explicit PreparedSimplicialLdlt(std::shared_ptr<const fem::CsrMatrix> matrix)
      : matrix_(std::move(matrix)) {
    if (matrix_->rows() == 0)
      return; // nothing to factorize: every solve returns an empty vector
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
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  Ldlt ldlt_;
};

} // namespace

std::unique_ptr<const PreparedLinearSolver>
SimplicialLdlt::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedSimplicialLdlt>(std::move(matrix));
}

} // namespace pde2d::solve
