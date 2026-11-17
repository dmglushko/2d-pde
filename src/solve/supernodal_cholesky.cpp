#include "pde2d/solve/supernodal_cholesky.hpp"

#include "factorization_threads.hpp"
#include "suitesparse.hpp"

#include <cholmod.h>

#include <cstddef>
#include <algorithm>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::solve {

namespace {

constexpr const char *solver_name = "supernodal_cholesky";

detail::DirectOptions direct(const SupernodalCholeskyOptions &options) {
  return {.residual_tolerance = options.residual_tolerance,
          .threads = options.threads};
}

// A cholmod_common for one prepared solver: CHOLMOD's settings and
// workspace. It is used only under a FactorizationThreads scope, whose lock
// also keeps two solves of one prepared solver from using it at once.
class Common {
public:
  explicit Common(int threads) {
    if (!cholmod_l_start(&common_))
      throw std::runtime_error(std::string(solver_name) +
                               ": CHOLMOD could not start");
    common_.print = 0; // errors become exceptions here, not printed text
    common_.nthreads_max = threads;
    common_.supernodal = CHOLMOD_SUPERNODAL; // always, whatever the size
  }
  ~Common() { cholmod_l_finish(&common_); }
  Common(const Common &) = delete;
  Common &operator=(const Common &) = delete;

  cholmod_common *get() { return &common_; }

private:
  cholmod_common common_{};
};

std::string status_text(int status) {
  switch (status) {
  case CHOLMOD_OUT_OF_MEMORY:
    return "out of memory";
  case CHOLMOD_TOO_LARGE:
    return "the problem is too large";
  case CHOLMOD_INVALID:
    return "invalid input";
  default:
    return std::format("CHOLMOD status {}", status);
  }
}

class PreparedSupernodalCholesky final : public PreparedLinearSolver {
public:
  PreparedSupernodalCholesky(std::shared_ptr<const fem::CsrMatrix> matrix,
                             const SupernodalCholeskyOptions &options)
      : matrix_(std::move(matrix)), options_(options),
        threads_(detail::resolve_threads(options.threads)),
        common_(std::make_unique<Common>(threads_)) {
    const std::size_t n = matrix_->rows();
    if (n == 0)
      return; // nothing to factorize: every solve returns an empty vector
    const detail::FactorizationThreads scope(threads_, threads_);
    cholmod_common *c = common_->get();
    // The lower triangle of the matrix (stype 1 of its transpose).
    cholmod_sparse a = detail::transposed_view(*matrix_, 1);
    factor_ = cholmod_l_analyze(&a, c);
    if (!factor_)
      throw std::runtime_error(std::format(
          "{}: the analysis failed ({})", solver_name, status_text(c->status)));
    cholmod_l_factorize(&a, factor_, c);
    if (c->status < CHOLMOD_OK) {
      free_factor();
      throw std::runtime_error(std::format("{}: the factorization failed ({})",
                                           solver_name,
                                           status_text(c->status)));
    }
    // CHOLMOD_NOT_POSDEF: a pivot that is not positive; factor_->minor is
    // its column (in the ordering), n when all pivots are positive.
    if (c->status == CHOLMOD_NOT_POSDEF ||
        static_cast<std::size_t>(factor_->minor) < n) {
      const auto column = static_cast<std::size_t>(factor_->minor);
      free_factor();
      throw std::runtime_error(std::format(
          "{}: the matrix is not positive definite (the factorization "
          "stopped at column {} of {}); for a negative definite matrix write "
          "the equation with the opposite sign, for an indefinite one use "
          "simplicial_ldlt or a LU solver",
          solver_name, column, n));
    }
    // (min diag L / max diag L)^2: the ratio of the smallest pivot to the
    // largest, as for an LDL^T factorization.
    const double rcond = cholmod_l_rcond(factor_, c);
    if (detail::singular_to_working_precision(rcond, n)) {
      free_factor();
      throw std::runtime_error(std::format(
          "{}: the matrix is singular to working precision (its smallest "
          "pivot is {} times the largest, below n machine epsilons)",
          solver_name, rcond));
    }
  }

  ~PreparedSupernodalCholesky() override { free_factor(); }
  PreparedSupernodalCholesky(const PreparedSupernodalCholesky &) = delete;
  PreparedSupernodalCholesky &
  operator=(const PreparedSupernodalCholesky &) = delete;

  LinearSolveResult
  solve(std::span<const double> right_hand_side) const override {
    detail::check_right_hand_side(matrix_->rows(), right_hand_side,
                                  solver_name);
    LinearSolveResult result;
    const std::size_t n = matrix_->rows();
    if (n == 0)
      return result;
    result.threads = static_cast<std::size_t>(threads_);
    // Allocated before CHOLMOD's result exists, so nothing can throw
    // between its allocation and its release (review finding).
    result.values.resize(n);
    {
      const detail::FactorizationThreads scope(threads_, threads_);
      cholmod_common *c = common_->get();
      // CHOLMOD reads b through a non-const pointer: give it a copy.
      std::vector<double> b(right_hand_side.begin(), right_hand_side.end());
      cholmod_dense dense_b{};
      dense_b.nrow = n;
      dense_b.ncol = 1;
      dense_b.nzmax = n;
      dense_b.d = n;
      dense_b.x = b.data();
      dense_b.xtype = CHOLMOD_REAL;
      dense_b.dtype = CHOLMOD_DOUBLE;
      cholmod_dense *x = cholmod_l_solve(CHOLMOD_A, factor_, &dense_b, c);
      if (!x)
        throw std::runtime_error(std::format(
            "{}: the solve failed ({})", solver_name, status_text(c->status)));
      const auto *values = static_cast<const double *>(x->x);
      std::copy(values, values + n, result.values.begin());
      cholmod_l_free_dense(&x, c);
    }
    detail::check_direct_result(result, *matrix_, right_hand_side,
                                options_.residual_tolerance, solver_name);
    return result;
  }

private:
  void free_factor() {
    if (factor_)
      cholmod_l_free_factor(&factor_, common_->get());
  }

  std::shared_ptr<const fem::CsrMatrix> matrix_;
  SupernodalCholeskyOptions options_;
  int threads_; // options_.threads, 0 resolved
  // A pointer, so that the const solve() can use CHOLMOD's workspace.
  std::unique_ptr<Common> common_;
  cholmod_factor *factor_ = nullptr;
};

} // namespace

SupernodalCholesky::SupernodalCholesky(SupernodalCholeskyOptions options)
    : options_(options) {
  detail::check_options(direct(options_), name());
}

std::unique_ptr<const PreparedLinearSolver> SupernodalCholesky::prepare(
    std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedSupernodalCholesky>(std::move(matrix),
                                                            options_);
}

} // namespace pde2d::solve
