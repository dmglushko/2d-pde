#include "pde2d/solve/supernodal_cholesky.hpp"

#include "factorization_threads.hpp"
#include "pivot_ratio.hpp"
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

// Whether every diagonal entry of `matrix` is negative (a missing one is
// zero). Such a symmetric matrix can only be negative definite or
// indefinite: its negation is factored.
bool diagonal_is_negative(const fem::CsrMatrix &matrix) {
  for (std::size_t i = 0; i < matrix.rows(); ++i)
    if (!(matrix.entry(i, i) < 0.0))
      return false;
  return true;
}

// The warning of every solve with a negated factorization
// (LinearSolveResult::messages).
constexpr const char *negated_message =
    "supernodal_cholesky: the matrix is negative definite (every diagonal "
    "entry is negative, as from an equation written with positive a_xx and "
    "a_yy), so -A was factored and -A x = -b solved; the answer is "
    "unchanged";

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
    // A negative diagonal: factor -A, a copy of the values with their signs
    // flipped (exact), on the same pattern; it lives only until the
    // factorization is done.
    negated_ = diagonal_is_negative(*matrix_);
    fem::CsrMatrix negation;
    if (negated_) {
      negation.pattern = matrix_->pattern;
      negation.values.reserve(matrix_->values.size());
      for (const double value : matrix_->values)
        negation.values.push_back(-value);
    }
    const fem::CsrMatrix &factored = negated_ ? negation : *matrix_;
    const detail::FactorizationThreads scope(threads_, threads_);
    cholmod_common *c = common_->get();
    // The lower triangle of the matrix (stype 1 of its transpose).
    cholmod_sparse a = detail::transposed_view(factored, 1);
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
    // CHOLMOD_NOT_POSDEF: a pivot that is not positive (zero, also
    // exactly zero for a singular matrix, or negative); factor_->minor is
    // its column (in the ordering), n when all pivots are positive.
    if (c->status == CHOLMOD_NOT_POSDEF ||
        static_cast<std::size_t>(factor_->minor) < n) {
      const auto column = static_cast<std::size_t>(factor_->minor);
      free_factor();
      // A negative definite matrix never gets here (its diagonal is
      // negative, and -A is positive definite), so the advice is only for
      // an indefinite or singular one.
      if (negated_)
        throw std::runtime_error(std::format(
            "{}: the matrix is not positive definite, and not negative "
            "definite although its diagonal is negative (the factorization "
            "of -A stopped at column {} of {}, at a pivot that is zero or "
            "negative); for an indefinite matrix use simplicial_ldlt or a LU "
            "solver",
            solver_name, column, n));
      throw std::runtime_error(std::format(
          "{}: the matrix is not positive definite (the factorization "
          "stopped at column {} of {}, at a pivot that is zero or negative); "
          "for an indefinite matrix use simplicial_ldlt or a LU solver",
          solver_name, column, n));
    }
    // (min diag L / max diag L)^2: the ratio of the smallest pivot to the
    // largest, as for an L D L^T factorization (of -A, the same in
    // magnitude). Reported, not judged: a tiny positive pivot (a pure
    // Neumann problem, a very ill-conditioned one) is accepted, and the
    // backward error of each solve is checked.
    pivot_ratio_ = cholmod_l_rcond(factor_, c);
    pivot_warning_ = detail::pivot_ratio_warning(solver_name, pivot_ratio_);
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
    result.pivot_ratio = pivot_ratio_;
    if (negated_)
      result.messages.emplace_back(negated_message);
    if (!pivot_warning_.empty())
      result.messages.push_back(pivot_warning_);
    // Allocated before CHOLMOD's result exists, so nothing can throw
    // between its allocation and its release (review finding).
    result.values.resize(n);
    {
      const detail::FactorizationThreads scope(threads_, threads_);
      cholmod_common *c = common_->get();
      // CHOLMOD reads b through a non-const pointer: give it a copy, -b
      // when -A was factored ((-A) x = -b has the same x; exact).
      std::vector<double> b(right_hand_side.begin(), right_hand_side.end());
      if (negated_)
        for (double &value : b)
          value = -value;
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
    // Checked against the matrix and right-hand side as given.
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
  double pivot_ratio_ = 0.0;  // 0 for an empty matrix
  bool negated_ = false;      // factor_ is that of -A
  std::string pivot_warning_; // empty unless pivot_ratio_ is tiny
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
