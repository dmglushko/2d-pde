#include "pde2d/solve/multifrontal_lu.hpp"

#include "factorization_threads.hpp"
#include "suitesparse.hpp"

#include <ParU.h>

#include <cstddef>
#include <cstdint>
#include <format>
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
    for (auto s = pattern.row_offsets[i]; s < pattern.row_offsets[i + 1];
         ++s) {
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
    // ParU runs its own OpenMP threads, and its dense updates split among
    // them, calling the BLAS from each; this SuiteSparse is built for a
    // generic BLAS, so ParU does not set the BLAS threads itself. The BLAS
    // runs in one thread, or every one of ParU's threads would start a
    // pool of its own.
    const detail::FactorizationThreads scope(threads_, 1);
    try {
      check(ParU_InitControl(&control_), "setting up");
      check(ParU_Set(PARU_CONTROL_MAX_THREADS,
                     static_cast<std::int64_t>(threads_), control_),
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
      // min |diag U| / max |diag U| of the scaled matrix.
      double rcond = 0.0;
      check(ParU_Get(symbolic_, numeric_, PARU_GET_RCOND_ESTIMATE, &rcond,
                     control_),
            "reading the pivots");
      if (detail::singular_to_working_precision(rcond, n))
        throw std::runtime_error(std::format(
            "{}: the matrix is singular to working precision (its smallest "
            "pivot is {} times the largest, below n machine epsilons)",
            solver_name, rcond));
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
    {
      const detail::FactorizationThreads scope(threads_, 1);
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
          "{}: the matrix is singular (a pivot is exactly zero)",
          solver_name));
    if (info != PARU_SUCCESS)
      throw std::runtime_error(
          std::format("{}: {} ({})", solver_name, what, info_text(info)));
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
