#include "pde2d/solve/multifrontal_lu.hpp"

#include "factorization_threads.hpp"
#include "suitesparse.hpp"

#include <ParU.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::solve {

namespace {

constexpr const char *solver_name = "multifrontal_lu";

detail::DirectOptions direct(const MultifrontalLuOptions &options) {
  return {.residual_tolerance = options.residual_tolerance,
          .threads = options.threads};
}

std::string info_text(ParU_Info info) {
  switch (info) {
  case PARU_OUT_OF_MEMORY:
    return "out of memory";
  case PARU_INVALID:
    return "invalid input";
  case PARU_SINGULAR:
    return "the matrix is singular";
  case PARU_TOO_LARGE:
    return "the problem is too large for the BLAS";
  default:
    return std::format("ParU status {}", static_cast<int>(info));
  }
}

// The matrix in compressed column form (ParU has no transposed solve, so
// our rows cannot serve as its columns). Counting sort by column: within
// each column the rows come out sorted.
struct ColumnMatrix {
  std::vector<std::int64_t> offsets, rows;
  std::vector<double> values;
};

ColumnMatrix columns_of(const fem::CsrMatrix &matrix) {
  const auto &pattern = *matrix.pattern;
  const std::size_t n = pattern.columns;
  ColumnMatrix c;
  c.offsets.assign(n + 1, 0);
  for (const auto column : pattern.column_indices)
    ++c.offsets[static_cast<std::size_t>(column) + 1];
  for (std::size_t j = 0; j < n; ++j)
    c.offsets[j + 1] += c.offsets[j];
  c.rows.resize(pattern.nonzeros());
  c.values.resize(pattern.nonzeros());
  std::vector<std::int64_t> next(c.offsets.begin(), c.offsets.end() - 1);
  for (std::size_t i = 0; i < pattern.rows; ++i)
    for (auto s = pattern.row_offsets[i]; s < pattern.row_offsets[i + 1]; ++s) {
      const auto slot = static_cast<std::size_t>(s);
      const auto at = static_cast<std::size_t>(
          next[static_cast<std::size_t>(pattern.column_indices[slot])]++);
      c.rows[at] = static_cast<std::int64_t>(i);
      c.values[at] = matrix.values[slot];
    }
  return c;
}

class PreparedMultifrontalLu final : public PreparedLinearSolver {
public:
  PreparedMultifrontalLu(std::shared_ptr<const fem::CsrMatrix> matrix,
                         const MultifrontalLuOptions &options)
      : matrix_(std::move(matrix)), options_(options),
        threads_(detail::resolve_threads(options.threads)) {
    const std::size_t n = matrix_->rows();
    if (n == 0)
      return; // nothing to factorize: every solve returns an empty vector
    // ParU runs `threads_` OpenMP threads (independent fronts at once, and
    // large dense updates split into tasks), each calling the BLAS. This
    // SuiteSparse is built for a generic BLAS, so ParU does not set the
    // BLAS threads itself: the scope gives OpenBLAS the same count.
    // OpenBLAS (pthreads) has one pool that its callers share, so at most
    // about twice `threads_` threads are busy at once; measured on 2D and
    // 3D problems with 4 threads, this is faster than a one-thread BLAS
    // (factorization 0.78 s instead of 0.87 s in 2D, 2.2 s instead of
    // 3.0 s in 3D).
    const detail::FactorizationThreads scope(threads_, threads_);
    try {
      check(ParU_InitControl(&control_), "setting up");
      check(ParU_Set(PARU_CONTROL_MAX_THREADS,
                     static_cast<std::int64_t>(threads_), control_),
            "setting up");
      // ParU's default, set here because pivot_ratio() relies on it: each
      // row is divided by its largest absolute entry.
      check(ParU_Set(PARU_CONTROL_PRESCALE,
                     static_cast<std::int64_t>(PARU_PRESCALE_MAX), control_),
            "setting up");
      ColumnMatrix columns = columns_of(*matrix_);
      cholmod_sparse a{};
      a.nrow = n;
      a.ncol = n;
      a.nzmax = columns.values.size();
      a.p = columns.offsets.data();
      a.i = columns.rows.data();
      a.x = columns.values.data();
      a.stype = 0;
      a.itype = CHOLMOD_LONG;
      a.xtype = CHOLMOD_REAL;
      a.dtype = CHOLMOD_DOUBLE;
      a.sorted = 1;
      a.packed = 1;
      check(ParU_Analyze(&a, &symbolic_, control_), "the analysis failed");
      check(ParU_Factorize(&a, symbolic_, &numeric_, control_),
            "the factorization failed");
      pivot_ratio_ = pivot_ratio();
    } catch (...) {
      release();
      throw;
    }
  }

  ~PreparedMultifrontalLu() override { release(); }
  PreparedMultifrontalLu(const PreparedMultifrontalLu &) = delete;
  PreparedMultifrontalLu &operator=(const PreparedMultifrontalLu &) = delete;

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
    {
      const detail::FactorizationThreads scope(threads_, threads_);
      // ParU reads b through a non-const pointer: give it a copy.
      std::vector<double> b(right_hand_side.begin(), right_hand_side.end());
      result.values.resize(n);
      check(ParU_Solve(symbolic_, numeric_, b.data(), result.values.data(),
                       control_),
            "the solve failed");
    }
    detail::check_direct_result(result, *matrix_, right_hand_side,
                                options_.residual_tolerance, solver_name);
    return result;
  }

private:
  static void check(ParU_Info info, const char *what) {
    if (info == PARU_SINGULAR)
      throw std::runtime_error(std::format(
          "{}: the matrix is singular (a pivot is exactly zero)", solver_name));
    if (info != PARU_SUCCESS)
      throw std::runtime_error(
          std::format("{}: {} ({})", solver_name, what, info_text(info)));
  }

  // The ratio of the smallest pivot |U_kk| of the row-scaled matrix to the
  // largest, over all pivots; throws std::runtime_error when one is exactly
  // zero. ParU's own figures (PARU_GET_MIN_UDIAG, PARU_GET_MAX_UDIAG and
  // their ratio PARU_GET_RCOND_ESTIMATE) cover only the pivots of the
  // fronts; the singletons (a column, then a row, with one entry left),
  // eliminated before the fronts, are added here. A singleton's pivot is
  // an entry of the scaled matrix that no elimination changed: entry
  // (P[k], Q[k]) divided by its row's scale factor, for the first
  // `singletons` positions k of ParU's row and column orders P and Q.
  // ParU checks the pattern, not the values, for singletons, so an
  // explicitly stored zero can be a singleton's pivot. (With threads, ParU
  // starts the smallest front pivot at 1 instead of at the first pivot, so
  // the ratio can differ between thread counts when every front pivot of
  // the scaled matrix is above 1.)
  double pivot_ratio() const {
    const std::size_t n = matrix_->rows();
    std::int64_t row_singletons = 0, column_singletons = 0;
    check(ParU_Get(symbolic_, numeric_, PARU_GET_NROW_SINGLETONS,
                   &row_singletons, control_),
          "reading the pivots");
    check(ParU_Get(symbolic_, numeric_, PARU_GET_NCOL_SINGLETONS,
                   &column_singletons, control_),
          "reading the pivots");
    const auto singletons =
        static_cast<std::size_t>(row_singletons + column_singletons);
    double smallest = std::numeric_limits<double>::infinity();
    double largest = 0.0;
    if (singletons > 0) {
      std::vector<std::int64_t> p(n), q(n);
      std::vector<double> scale(n);
      check(ParU_Get(symbolic_, numeric_, PARU_GET_P, p.data(), control_),
            "reading the pivots");
      check(ParU_Get(symbolic_, numeric_, PARU_GET_Q, q.data(), control_),
            "reading the pivots");
      check(ParU_Get(symbolic_, numeric_, PARU_GET_ROW_SCALE_FACTORS,
                     scale.data(), control_),
            "reading the pivots");
      for (std::size_t k = 0; k < singletons; ++k) {
        const auto row = static_cast<std::size_t>(p[k]);
        const double pivot = std::abs(entry(*matrix_, row, q[k])) / scale[row];
        smallest = std::min(smallest, pivot);
        largest = std::max(largest, pivot);
      }
    }
    // Both -1 without fronts (a matrix of singletons only).
    double front_smallest = 0.0, front_largest = 0.0;
    check(ParU_Get(symbolic_, numeric_, PARU_GET_MIN_UDIAG, &front_smallest,
                   control_),
          "reading the pivots");
    check(ParU_Get(symbolic_, numeric_, PARU_GET_MAX_UDIAG, &front_largest,
                   control_),
          "reading the pivots");
    if (front_largest >= 0.0) {
      smallest = std::min(smallest, front_smallest);
      largest = std::max(largest, front_largest);
    }
    if (!(smallest > 0.0))
      throw std::runtime_error(std::format(
          "{}: the matrix is singular (a pivot is exactly zero)", solver_name));
    return smallest / largest;
  }

  // The value in slot (row, column) of `matrix`; 0 when it has no slot.
  static double entry(const fem::CsrMatrix &matrix, std::size_t row,
                      std::int64_t column) {
    const auto &pattern = *matrix.pattern;
    const auto first =
                   pattern.column_indices.begin() + pattern.row_offsets[row],
               last = pattern.column_indices.begin() +
                      pattern.row_offsets[row + 1];
    const auto at = std::lower_bound(first, last, column);
    if (at == last || *at != column)
      return 0.0;
    return matrix
        .values[static_cast<std::size_t>(at - pattern.column_indices.begin())];
  }

  void release() {
    if (numeric_)
      ParU_FreeNumeric(&numeric_, control_);
    if (symbolic_)
      ParU_FreeSymbolic(&symbolic_, control_);
    if (control_)
      ParU_FreeControl(&control_);
  }

  std::shared_ptr<const fem::CsrMatrix> matrix_;
  MultifrontalLuOptions options_;
  int threads_; // options_.threads, 0 resolved
  ParU_Control control_ = nullptr;
  ParU_Symbolic symbolic_ = nullptr;
  ParU_Numeric numeric_ = nullptr;
  double pivot_ratio_ = 0.0; // 0 for an empty matrix
};

} // namespace

MultifrontalLu::MultifrontalLu(MultifrontalLuOptions options)
    : options_(options) {
  detail::check_options(direct(options_), name());
}

std::unique_ptr<const PreparedLinearSolver>
MultifrontalLu::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const PreparedMultifrontalLu>(std::move(matrix),
                                                        options_);
}

} // namespace pde2d::solve
