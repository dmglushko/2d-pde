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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
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

// Whether a factorization of an n x n matrix whose smallest pivot is
// `rcond` times its largest counts as singular: below n machine epsilons
// the smallest pivot is rounding noise. A singular matrix factorized in
// floating point rarely meets an exact zero pivot; its last pivot is a
// few epsilons times n instead (a pure Neumann problem: about 1e-17 n),
// while a matrix from a well-posed problem keeps pivots near 1/n.
inline bool singular_to_working_precision(double rcond, std::size_t n) {
  return !(rcond >=
           static_cast<double>(n) * std::numeric_limits<double>::epsilon());
}

// The checks after a direct solve: the result is finite and its relative
// residual is at most `tolerance`. Sets result.residual; throws
// std::runtime_error naming `solver` otherwise.
inline void check_direct_result(LinearSolveResult &result,
                                const fem::CsrMatrix &matrix,
                                std::span<const double> right_hand_side,
                                double tolerance, const std::string &solver) {
  if (!all_finite(result.values))
    throw std::runtime_error(
        solver +
        ": the solution is not finite (the matrix is singular or nearly so)");
  result.residual = relative_residual(matrix, right_hand_side, result.values);
  if (!(result.residual <= tolerance))
    throw std::runtime_error(std::format(
        "{}: the relative residual {} is above the tolerance {} (the matrix "
        "is nearly singular)",
        solver, result.residual, tolerance));
}

} // namespace pde2d::solve::detail
