#include "pde2d/fem/csr_matrix.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::fem {

std::optional<std::size_t> SparsityPattern::find(std::size_t row,
                                                 std::size_t column) const {
  if (row >= rows || column >= columns)
    throw std::invalid_argument(
        std::format("Entry ({}, {}) is outside the {} x {} matrix", row, column,
                    rows, columns));
  const auto begin = column_indices.begin() + row_offsets[row];
  const auto end = column_indices.begin() + row_offsets[row + 1];
  const auto found = std::lower_bound(begin, end, static_cast<Index>(column));
  if (found == end || *found != static_cast<Index>(column))
    return std::nullopt;
  return static_cast<std::size_t>(found - column_indices.begin());
}

double CsrMatrix::entry(std::size_t row, std::size_t column) const {
  if (!pattern)
    throw std::invalid_argument(
        std::format("Entry ({}, {}) is outside the empty matrix", row, column));
  const auto slot = pattern->find(row, column);
  return slot ? values[*slot] : 0.0;
}

std::vector<double> CsrMatrix::multiply(std::span<const double> vector) const {
  if (vector.size() != columns())
    throw std::invalid_argument(
        std::format("Cannot multiply a {} x {} matrix by a vector of size {}",
                    rows(), columns(), vector.size()));
  std::vector<double> product(rows(), 0.0);
  for (std::size_t row = 0; row < rows(); ++row) {
    double sum = 0.0;
    for (auto slot = pattern->row_offsets[row];
         slot < pattern->row_offsets[row + 1]; ++slot) {
      const auto s = static_cast<std::size_t>(slot);
      sum += values[s] *
             vector[static_cast<std::size_t>(pattern->column_indices[s])];
    }
    product[row] = sum;
  }
  return product;
}

namespace {

void check_values(const CsrMatrix &matrix, const char *function) {
  const std::size_t slots = matrix.pattern ? matrix.pattern->nonzeros() : 0;
  if (matrix.values.size() != slots)
    throw std::invalid_argument(
        std::format("{}: the matrix has {} values for {} slots", function,
                    matrix.values.size(), slots));
}

} // namespace

bool has_only_zero_entries(const CsrMatrix &matrix) {
  check_values(matrix, "has_only_zero_entries");
  return std::ranges::all_of(matrix.values,
                             [](double value) { return value == 0.0; });
}

bool is_symmetric(const CsrMatrix &matrix, double relative_tolerance) {
  check_values(matrix, "is_symmetric");
  if (!std::isfinite(relative_tolerance) || relative_tolerance < 0.0)
    throw std::invalid_argument(std::format(
        "is_symmetric: the relative tolerance {} must be finite and not "
        "negative",
        relative_tolerance));
  if (matrix.rows() != matrix.columns())
    return false;
  if (!matrix.pattern)
    return true; // 0 x 0
  // The scale the tolerance is relative to. A value that is not finite
  // makes the matrix non-symmetric (the comparisons below would be false
  // for a NaN, and inf - inf is a NaN).
  double largest = 0.0;
  for (const double value : matrix.values) {
    if (!std::isfinite(value))
      return false;
    largest = std::max(largest, std::abs(value));
  }
  const double allowed = relative_tolerance * largest;
  // Every slot (i, j) against its mirror (j, i). Visiting every slot also
  // covers the mirrors that are missing from the pattern, which count as
  // 0, from both sides.
  const SparsityPattern &pattern = *matrix.pattern;
  for (std::size_t row = 0; row < pattern.rows; ++row)
    for (auto slot = pattern.row_offsets[row];
         slot < pattern.row_offsets[row + 1]; ++slot) {
      const auto s = static_cast<std::size_t>(slot);
      const auto column = static_cast<std::size_t>(pattern.column_indices[s]);
      if (column == row)
        continue;
      const auto mirror = pattern.find(column, row);
      const double other = mirror ? matrix.values[*mirror] : 0.0;
      if (std::abs(matrix.values[s] - other) > allowed)
        return false;
    }
  return true;
}

} // namespace pde2d::fem
