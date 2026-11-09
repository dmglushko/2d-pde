#pragma once
// The shared linear solve driver, independent of the element type:
//
//   PreparedLinearSolver(A, options)  validation, rank check, backend setup
//   prepared.solve(b)                 backend solve + true-residual check
//   prepared.solve(b, guess)          same, starting from an initial guess
//   solve_linear(A, b, options)       one-shot form
//
// A backend (LinearSolverBackend, linear_solver_types.hpp) owns only the
// matrix-dependent work of one method: factorization or preconditioner
// setup, and the solve itself. PreparedLinearSolver keeps everything shared:
// option and matrix validation, the numerical rank check, tolerances, the
// true-residual check, timings and the reuse test. The built-in backends
// (detail/eigen_backends.hpp) are selected by SolverOptions::method;
// SolverOptions::backend plugs in a user backend without editing the library
// (see EXTENDING.md).
#include "fem/constraints.hpp"
#include "fem/detail/eigen_backends.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/system.hpp"
#include "fem/timing.hpp"
#include <Eigen/SparseQR>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
// Owns its matrix snapshot and all matrix-dependent preparation. Calls to solve
// are sequential; each returned report contains only that RHS's work.
class PreparedLinearSolver {
  SparseMatrix matrix_;
  SolverOptions options_;
  SolverTimings preparation_;
  std::string failure_;
  std::unique_ptr<LinearSolverBackend> backend_;

public:
  PreparedLinearSolver(const SparseMatrix &input,
                       const SolverOptions &options = {})
      : options_(options) {
    const auto started = timing::Clock::now();
    matrix_ = input;
    solver_detail::validate(options_);
    backend_ = solver_detail::make_backend(options_);
    solver_detail::validate(matrix_, Vector::Zero(matrix_.rows()));
    matrix_.makeCompressed();
    preparation_.preparations = 1;
    preparation_.validation_seconds = timing::seconds_since(started);
    auto phase = timing::Clock::now();
    if (options_.validation == SolverValidation::Checked) {
      Eigen::SparseQR<SparseMatrix, Eigen::COLAMDOrdering<int>> rank;
      rank.compute(matrix_);
      ++preparation_.rank_checks;
      preparation_.rank_check_seconds = timing::seconds_since(phase);
      if (rank.info() != Eigen::Success || rank.rank() != matrix_.rows())
        failure_ = "Singular or numerically rank-deficient linear system";
    }
    if (failure_.empty()) {
      phase = timing::Clock::now();
      ++preparation_.factorizations;
      failure_ = backend_->prepare(matrix_, options_);
      preparation_.factorization_seconds = timing::seconds_since(phase);
    }
    preparation_.total_seconds = timing::seconds_since(started);
  }
  const SparseMatrix &matrix() const noexcept { return matrix_; }
  const SolverOptions &options() const noexcept { return options_; }
  const SolverTimings &preparation_timings() const noexcept {
    return preparation_;
  }
  // Compare actual coefficients (including explicit zeros), never identities or
  // merely sparsity. A conservative false miss is safe; an approximate hit
  // isn't.
  bool matches(const SparseMatrix &a, const SolverOptions &o) const {
    if (o != options_ || a.rows() != matrix_.rows() ||
        a.cols() != matrix_.cols())
      return false;
    for (int col = 0; col < a.outerSize(); ++col) {
      SparseMatrix::InnerIterator left(a, col), right(matrix_, col);
      for (; left && right; ++left, ++right)
        if (left.row() != right.row() || left.value() != right.value())
          return false;
      if (left || right)
        return false;
    }
    return true;
  }
  LinearSolution solve(const Vector &b) { return solve_from(b, nullptr); }
  // Start from `guess` (one finite entry per row). Iterative backends use it;
  // direct ones ignore it (SolverReport::initial_guess says which).
  LinearSolution solve(const Vector &b, const Vector &guess) {
    return solve_from(b, &guess);
  }

private:
  LinearSolution solve_from(const Vector &b, const Vector *guess) {
    const auto started = timing::Clock::now();
    if (b.size() != matrix_.rows() || !b.allFinite())
      throw std::invalid_argument("Invalid linear RHS");
    if (guess && (guess->size() != matrix_.rows() || !guess->allFinite()))
      throw std::invalid_argument("Invalid initial guess");
    LinearSolution result{Vector::Zero(b.size()), {}};
    auto &r = result.report;
    r.timings.rhs_solves = 1;
    const double bn = b.stableNorm();
    r.residual_norm = bn;
    r.relative_residual = bn > 0 ? 1. : 0.;
    r.timings.validation_seconds = timing::seconds_since(started);
    auto finish = [&]() {
      r.timings.total_seconds = timing::seconds_since(started);
      return std::move(result);
    };
    if (!failure_.empty()) {
      r.message = failure_;
      return finish();
    }
    const double target =
        std::max(options_.absolute_tolerance, options_.relative_tolerance * bn);
    const double tolerance = bn > 0 ? target / bn : options_.relative_tolerance;
    auto phase = timing::Clock::now();
    auto solved = guess ? backend_->solve_with_guess(b, tolerance, *guess)
                        : backend_->solve(b, tolerance);
    if (guess)
      r.initial_guess =
          solved.used_guess ? InitialGuess::Used : InitialGuess::Ignored;
    result.values = std::move(solved.values);
    r.iterations = solved.iterations;
    r.timings.solve_seconds = timing::seconds_since(phase);
    if (!solved.failure.empty()) {
      r.message = std::move(solved.failure);
      return finish();
    }
    if (result.values.size() != b.size())
      throw std::runtime_error("Linear solver backend returned a solution "
                               "of the wrong size");
    phase = timing::Clock::now();
    r.residual_norm = (matrix_ * result.values - b).stableNorm();
    r.relative_residual = bn > 0 ? r.residual_norm / bn : r.residual_norm;
    r.converged = result.values.allFinite() && std::isfinite(r.residual_norm) &&
                  r.residual_norm <= target;
    r.message = r.converged ? "Converged"
                            : "Linear solver did not converge to requested "
                              "true residual tolerance";
    r.timings.residual_seconds = timing::seconds_since(phase);
    return finish();
  }
};
namespace solver_detail {
inline LinearSolution solve_linear(const SparseMatrix &input, const Vector &b,
                                   const SolverOptions &o,
                                   const Vector *guess) {
  const auto started = timing::Clock::now();
  validate(o);
  validate(input, b);
  PreparedLinearSolver prepared(input, o);
  auto result = guess ? prepared.solve(b, *guess) : prepared.solve(b);
  result.report.timings += prepared.preparation_timings();
  result.report.timings.total_seconds = timing::seconds_since(started);
  return result;
}
} // namespace solver_detail
inline LinearSolution solve_linear(const SparseMatrix &input, const Vector &b,
                                   const SolverOptions &o = {}) {
  return solver_detail::solve_linear(input, b, o, nullptr);
}
// Start an iterative backend from `guess`; direct backends ignore it.
inline LinearSolution solve_linear(const SparseMatrix &input, const Vector &b,
                                   const SolverOptions &o,
                                   const Vector &guess) {
  return solver_detail::solve_linear(input, b, o, &guess);
}
} // namespace pde2d::fem
