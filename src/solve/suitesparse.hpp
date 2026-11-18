#pragma once

// Private: what the two SuiteSparse solvers (SupernodalCholesky with
// CHOLMOD, MultifrontalLu with ParU) share. Includes CHOLMOD's header:
// only their .cpp files include this one.
//
// SuiteSparse's headers declare C functions (and, for ParU, C++ functions
// that are not inline) and macros; they define no inline function with
// external linkage whose code depends on OpenMP, so linking SuiteSparse
// privately cannot give a program a second, differently compiled copy of
// a function (the reason OpenMP is a public requirement of pde2d_solve,
// for Eigen's inline Eigen::nbThreads).

#include "threads.hpp"
#include "vectors.hpp"

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/solve/linear_solver.hpp"

#include <cholmod.h>

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace pde2d::solve::detail {

// Our pattern's index type is SuiteSparse's 64-bit index (CHOLMOD_LONG),
// so its arrays can be given to CHOLMOD without converting them.
static_assert(sizeof(fem::SparsityPattern::Index) == sizeof(std::int64_t) &&
              std::is_signed_v<fem::SparsityPattern::Index>);

// The options both solvers have.
struct DirectOptions {
  double residual_tolerance = 1e-8;
  std::size_t threads = 1;
};

// Throws std::invalid_argument, naming `solver`, unless 0 <
// residual_tolerance < 1 and threads <= max_threads.
inline void check_options(const DirectOptions &options,
                          const std::string &solver) {
  if (!(options.residual_tolerance > 0.0 && options.residual_tolerance < 1.0))
    throw std::invalid_argument(solver +
                                ": residual_tolerance must be in (0, 1)");
  check_threads(options.threads, solver);
}

// CHOLMOD's description of our CSR arrays as a compressed column matrix,
// without copying: the columns of that matrix are our rows, so it is the
// transpose of `matrix`. `stype` 0 uses every entry, 1 only the upper
// triangle of the transpose, which is the lower triangle of `matrix`.
// CHOLMOD's analysis and factorization only read the arrays (it sorts a
// matrix only when `sorted` is false; our rows are sorted and unique), so
// the const_casts never lead to a write.
inline cholmod_sparse transposed_view(const fem::CsrMatrix &matrix, int stype) {
  const auto &pattern = *matrix.pattern;
  cholmod_sparse a{};
  a.nrow = pattern.columns;
  a.ncol = pattern.rows;
  a.nzmax = pattern.nonzeros();
  a.p = const_cast<fem::SparsityPattern::Index *>(pattern.row_offsets.data());
  a.i =
      const_cast<fem::SparsityPattern::Index *>(pattern.column_indices.data());
  a.x = const_cast<double *>(matrix.values.data());
  a.stype = stype;
  a.itype = CHOLMOD_LONG;
  a.xtype = CHOLMOD_REAL;
  a.dtype = CHOLMOD_DOUBLE;
  a.sorted = 1;
  a.packed = 1;
  return a;
}

// The checks after a direct solve: the result is finite and its backward
// error |b - A x| / (|A| |x| + |b|) is at most `tolerance` (the
// relative residual is reported, not checked: for an accurate x it is
// about machine epsilon times |A| |x| / |b|, large for a large coefficient
// contrast). Sets result.residual and result.backward_error; throws
// std::runtime_error naming `solver` otherwise.
inline void check_direct_result(LinearSolveResult &result,
                                const fem::CsrMatrix &matrix,
                                std::span<const double> right_hand_side,
                                double tolerance, const std::string &solver) {
  if (!all_finite(result.values))
    throw std::runtime_error(
        solver +
        ": the solution is not finite (the matrix is singular or nearly so)");
  const auto measures =
      measure_residual(matrix, right_hand_side, result.values);
  result.residual = measures.relative_residual;
  result.backward_error = measures.backward_error;
  if (!(result.backward_error <= tolerance))
    throw std::runtime_error(std::format(
        "{}: the backward error {} is above the tolerance {} "
        "(residual_tolerance; the factorization lost accuracy on this "
        "matrix, pivot ratio {})",
        solver, result.backward_error, tolerance, result.pivot_ratio));
}

} // namespace pde2d::solve::detail
