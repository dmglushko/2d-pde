#pragma once

// Small vector and matrix helpers shared by the solvers and time methods.
// No Eigen here: the time methods work with fem::CsrMatrix directly.

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/solve/time_method.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::solve::detail {

// The largest |v_i|; 0 for an empty or zero vector, NaN when an entry is
// NaN (std::max alone would skip it), so a residual that is not finite
// gives a backward error that no tolerance accepts.
inline double max_abs(std::span<const double> v) {
  double largest = 0.0;
  for (const double x : v) {
    if (std::isnan(x))
      return x;
    largest = std::max(largest, std::abs(x));
  }
  return largest;
}

// The Euclidean norm. The entries are divided by the largest one before
// squaring, so a vector of tiny (1e-200) or huge (1e200) entries does not
// underflow to 0 or overflow to infinity.
inline double norm(std::span<const double> v) {
  const double largest = max_abs(v);
  if (largest == 0.0 || !std::isfinite(largest))
    return largest;
  double sum = 0.0;
  for (const double x : v)
    sum += (x / largest) * (x / largest);
  return largest * std::sqrt(sum);
}

// The infinity norm of a matrix: its largest absolute row sum,
// max_i sum_j |a_ij|; 0 for an empty matrix.
inline double max_row_sum(const fem::CsrMatrix &matrix) {
  const auto &pattern = *matrix.pattern;
  double largest = 0.0;
  for (std::size_t i = 0; i < pattern.rows; ++i) {
    double sum = 0.0;
    for (auto s = pattern.row_offsets[i]; s < pattern.row_offsets[i + 1]; ++s)
      sum += std::abs(matrix.values[static_cast<std::size_t>(s)]);
    largest = std::max(largest, sum);
  }
  return largest;
}

// The normwise backward error |r| / (|A| |x| + |b|) from the infinity
// norms of the residual r = b - A x, of A (its largest absolute row sum),
// of x and of b: the smallest relative change of A and b (in that norm)
// for which x is the exact solution. 0 when r is zero, also for b = 0 and
// x = 0; infinity when r is not zero but A x and b are (only possible for
// a zero matrix); NaN when |r| is NaN (the checks `!(error <= tolerance)`
// then reject it).
inline double backward_error(double residual_norm, double matrix_norm,
                             double x_norm, double b_norm) {
  if (residual_norm == 0.0)
    return 0.0;
  const double scale = matrix_norm * x_norm + b_norm;
  return scale > 0.0 ? residual_norm / scale
                     : std::numeric_limits<double>::infinity();
}

// What a solve reports about its residual r = b - A x, computed once.
struct ResidualMeasures {
  // |r| / |b| in the Euclidean norm, or |r| when b is zero.
  double relative_residual = 0.0;
  // backward_error() in the infinity norm.
  double backward_error = 0.0;
};

// The residual of x for matrix * x = b, in one pass over the matrix (the
// product and the absolute row sums together).
inline ResidualMeasures measure_residual(const fem::CsrMatrix &matrix,
                                         std::span<const double> b,
                                         std::span<const double> x) {
  const auto &pattern = *matrix.pattern;
  std::vector<double> r(pattern.rows);
  double matrix_norm = 0.0;
  for (std::size_t i = 0; i < pattern.rows; ++i) {
    double product = 0.0, row_sum = 0.0;
    for (auto s = pattern.row_offsets[i]; s < pattern.row_offsets[i + 1]; ++s) {
      const auto slot = static_cast<std::size_t>(s);
      const double a = matrix.values[slot];
      product += a * x[static_cast<std::size_t>(pattern.column_indices[slot])];
      row_sum += std::abs(a);
    }
    r[i] = b[i] - product;
    matrix_norm = std::max(matrix_norm, row_sum);
  }
  const double b_norm = norm(b);
  return {.relative_residual = b_norm > 0.0 ? norm(r) / b_norm : norm(r),
          .backward_error =
              backward_error(max_abs(r), matrix_norm, max_abs(x), max_abs(b))};
}

inline bool all_finite(std::span<const double> v) {
  for (const double x : v)
    if (!std::isfinite(x))
      return false;
  return true;
}

// The checks every LinearSolver::prepare makes.
inline void check_matrix(const std::shared_ptr<const fem::CsrMatrix> &matrix,
                         const std::string &solver) {
  if (!matrix)
    throw std::invalid_argument(solver + ": the matrix is null");
  if (!matrix->pattern)
    throw std::invalid_argument(solver + ": the matrix has no pattern");
  if (matrix->rows() != matrix->columns())
    throw std::invalid_argument(solver + ": the matrix is not square (" +
                                std::to_string(matrix->rows()) + " x " +
                                std::to_string(matrix->columns()) + ")");
  if (matrix->values.size() != matrix->pattern->nonzeros())
    throw std::invalid_argument(solver +
                                ": the matrix has not one value per slot");
}

// The check every PreparedLinearSolver::solve makes.
inline void check_right_hand_side(std::size_t rows,
                                  std::span<const double> right_hand_side,
                                  const std::string &solver) {
  if (right_hand_side.size() != rows)
    throw std::invalid_argument(solver + ": the right-hand side has " +
                                std::to_string(right_hand_side.size()) +
                                " values for " + std::to_string(rows) +
                                " rows");
}

// out += scale * v.
inline void add_scaled(std::vector<double> &out, double scale,
                       std::span<const double> v) {
  for (std::size_t i = 0; i < out.size(); ++i)
    out[i] += scale * v[i];
}

// out += scale * (free part of op) * v; nothing for a zero operator.
inline void add_product(std::vector<double> &out, double scale,
                        const fem::Operator &op, std::span<const double> v) {
  if (op.always_zero || scale == 0.0)
    return;
  add_scaled(out, scale, op.free.multiply(v));
}

// The right-hand side of the assembled problem, as a vector to add to.
inline std::vector<double> right_hand_side(const fem::AssembledProblem &p) {
  return {p.right_hand_side().begin(), p.right_hand_side().end()};
}

} // namespace pde2d::solve::detail
