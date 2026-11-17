#include "pde2d/solve/sparse_lu.hpp"

#include "eigen_threads.hpp"
#include "eigen_view.hpp"
#include "vectors.hpp"

#include <Eigen/OrderingMethods>
#include <Eigen/SparseLU>

#include <memory>
#include <stdexcept>
#include <utility>

namespace pde2d::solve {

namespace {

// SparseLU works on a column-major matrix with int indices (the index type
// of its COLAMD ordering), so the matrix is copied once into that format;
// the factors are a copy anyway.
using ColumnMajorMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using Lu = Eigen::SparseLU<ColumnMajorMatrix, Eigen::COLAMDOrdering<int>>;

class PreparedSparseLu final : public PreparedLinearSolver {
public:
  explicit PreparedSparseLu(std::shared_ptr<const fem::CsrMatrix> matrix)
      : matrix_(std::move(matrix)) {
    if (matrix_->rows() == 0)
      return; // nothing to factorize: every solve returns an empty vector
    // Direct solvers run in one thread (docs/decisions.md: their parallel
    // version needs a library still to be chosen).
    const detail::EigenThreads scope(1);
    const ColumnMajorMatrix a = detail::view(*matrix_);
    lu_.analyzePattern(a);
    lu_.factorize(a);
    if (lu_.info() != Eigen::Success)
      throw std::runtime_error("sparse_lu: the matrix is singular (" +
                               lu_.lastErrorMessage() + ")");
  }

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    detail::check_right_hand_side(matrix_->rows(), right_hand_side,
                                  "sparse_lu");
    LinearSolveResult result;
    if (matrix_->rows() == 0)
      return result;
    const detail::EigenThreads scope(1);
    const Eigen::VectorXd x = lu_.solve(detail::view(right_hand_side));
    if (lu_.info() != Eigen::Success)
      throw std::runtime_error("sparse_lu: the solve failed");
    result.values.assign(x.data(), x.data() + x.size());
    if (!detail::all_finite(result.values))
      throw std::runtime_error(
          "sparse_lu: the solution is not finite (the matrix is singular or "
          "nearly so)");
    result.residual =
        detail::relative_residual(*matrix_, right_hand_side, result.values);
    return result;
  }

private:
  std::shared_ptr<const fem::CsrMatrix> matrix_;
  // Eigen's solve() is const but the factorization object is not copyable;
  // it lives as long as the prepared solver.
  Lu lu_;
};

} // namespace

std::unique_ptr<const PreparedLinearSolver>
SparseLu::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedSparseLu>(std::move(matrix));
}

} // namespace pde2d::solve
