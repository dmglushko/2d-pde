#pragma once

// Small vector and matrix helpers shared by the solvers and time methods.
// No Eigen here: the time methods work with fem::CsrMatrix directly.

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/solve/time_method.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::solve::detail {

// The largest |v_i|; 0 for an empty or zero vector.
inline double max_abs(std::span<const double> v) {
  double largest = 0.0;
  for (const double x : v)
    largest = std::max(largest, std::abs(x));
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

// |b - A x| / |b|, or |b - A x| when b is zero.
inline double relative_residual(const fem::CsrMatrix &matrix,
                                std::span<const double> b,
                                std::span<const double> x) {
  std::vector<double> r = matrix.multiply(x);
  for (std::size_t i = 0; i < r.size(); ++i)
    r[i] = b[i] - r[i];
  const double b_norm = norm(b);
  return b_norm > 0.0 ? norm(r) / b_norm : norm(r);
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
  if (op.is_zero || scale == 0.0)
    return;
  add_scaled(out, scale, op.free.multiply(v));
}

// The right-hand side of the assembled problem, as a vector to add to.
inline std::vector<double> right_hand_side(const fem::AssembledProblem &p) {
  return {p.right_hand_side().begin(), p.right_hand_side().end()};
}

} // namespace pde2d::solve::detail
